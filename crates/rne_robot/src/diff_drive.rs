//! Differential drive robot spawning and kinematics metadata.

use crate::actuator::{ActuatorLimits, ActuatorTarget, ControlMode};
use crate::components::{Actuator, Joint, JointKind, JointLimits, Link, Robot, RobotId};
use bevy_ecs::prelude::Component;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, JointMotor, PhysicsMaterial, RevoluteJointDesc, RigidBody,
    RigidBodyType,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// How wheel commands move the diff-drive robot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiffDriveDriveMode {
    /// Analytic kinematics on the base link (legacy default).
    #[default]
    Kinematic,
    /// Rapier revolute joints with velocity motors on each wheel.
    JointDriven,
}

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
    /// Wheel actuation model.
    pub drive_mode: DiffDriveDriveMode,
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
            drive_mode: DiffDriveDriveMode::Kinematic,
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

    let half_track = config.track_width_m * 0.5;
    let wheel_offset = Vec3::new(
        0.0,
        -config.base_half_extents_m.y + config.wheel_radius_m,
        0.0,
    );
    let base_translation = base_translation_for_mode(config, wheel_offset.y);

    world.entity_mut(robot).insert(Robot {
        robot_id: RobotId::default(),
        model_name: config.model_name.clone(),
        base_link,
    });

    let base_body_type = match config.drive_mode {
        DiffDriveDriveMode::Kinematic => RigidBodyType::Kinematic,
        DiffDriveDriveMode::JointDriven => RigidBodyType::Dynamic,
    };

    world.entity_mut(base_link).insert((
        Link {
            robot,
            name: "base_link".into(),
        },
        Transform3::from_translation_rotation(base_translation, Quat::IDENTITY),
        RigidBody {
            body_type: base_body_type,
            mass_kg: 5.0,
            ..RigidBody::default()
        },
        Collider::cuboid(config.base_half_extents_m),
    ));

    for (wheel, name, x_offset, actuator_entity) in [
        (left_wheel, "left_wheel", -half_track, left_actuator),
        (right_wheel, "right_wheel", half_track, right_actuator),
    ] {
        let wheel_translation = base_translation + Vec3::new(x_offset, wheel_offset.y, 0.0);
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
            Transform3::from_translation_rotation(wheel_translation, Quat::IDENTITY),
        ));

        if config.drive_mode == DiffDriveDriveMode::JointDriven {
            world.entity_mut(wheel).insert((
                RigidBody {
                    body_type: RigidBodyType::Dynamic,
                    mass_kg: 0.5,
                    ..RigidBody::default()
                },
                Collider {
                    shape: ColliderShape::Capsule {
                        half_height_m: config.wheel_radius_m * 0.25,
                        radius_m: config.wheel_radius_m,
                    },
                    material: PhysicsMaterial {
                        friction: 1.2,
                        restitution: 0.0,
                    },
                    local_offset: Transform3::IDENTITY,
                    sensor: false,
                },
                RevoluteJointDesc {
                    parent: base_link,
                    axis: Vec3::Y,
                    anchor_parent_m: Vec3::new(x_offset, wheel_offset.y, 0.0),
                    anchor_child_m: Vec3::ZERO,
                    lower_rad: None,
                    upper_rad: None,
                },
                JointMotor::default(),
            ));
        }

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

fn base_translation_for_mode(config: &DiffDriveConfig, wheel_offset_y: f64) -> Vec3 {
    match config.drive_mode {
        DiffDriveDriveMode::Kinematic => config.initial_translation_m,
        DiffDriveDriveMode::JointDriven => {
            let base_y = config.wheel_radius_m - wheel_offset_y;
            Vec3::new(
                config.initial_translation_m.x,
                base_y,
                config.initial_translation_m.z,
            )
        }
    }
}

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

    #[test]
    fn joint_driven_spawn_attaches_physics_joints() {
        let mut world = World::new();
        let spawned = spawn_diff_drive_robot(
            &mut world,
            &DiffDriveConfig {
                drive_mode: DiffDriveDriveMode::JointDriven,
                ..DiffDriveConfig::default()
            },
        );

        assert!(world.get::<RevoluteJointDesc>(spawned.left_wheel).is_some());
        assert!(world.get::<JointMotor>(spawned.left_wheel).is_some());
        assert!(world.get::<RigidBody>(spawned.left_wheel).is_some());
        assert_eq!(
            world.get::<RigidBody>(spawned.base_link).unwrap().body_type,
            RigidBodyType::Dynamic
        );
    }
}
