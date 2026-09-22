//! Differential drive robot spawning and kinematics metadata.

use crate::actuator::{ActuatorLimits, ActuatorTarget, ControlMode};
use crate::components::{Actuator, Joint, JointKind, JointLimits, Link, Robot, RobotId};
use bevy_ecs::prelude::Component;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, ConvexCollider, FixedJointDesc, JointMotor, JointMotorGainModel,
    PhysicsMaterial, RevoluteJointDesc, RigidBody, RigidBodyType,
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
    // +X is forward, +Y is up, and the axle spans Z. Place the axle at
    // the chassis bottom so the wheel radius provides ground clearance.
    let wheel_offset = Vec3::new(0.0, -config.base_half_extents_m.y, 0.0);
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

    for (wheel, name, z_offset, actuator_entity) in [
        (left_wheel, "left_wheel", -half_track, left_actuator),
        (right_wheel, "right_wheel", half_track, right_actuator),
    ] {
        let wheel_translation = base_translation + Vec3::new(0.0, wheel_offset.y, z_offset);
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
                axis: Vec3::NEG_Z,
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
                    local_offset: Transform3::from_translation_rotation(
                        Vec3::ZERO,
                        Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                    ),
                    sensor: false,
                },
                // A capsule adds its radius to the axial width and penetrates the
                // chassis. A thin convex cylinder keeps the rolling radius while
                // fitting between the declared axle and chassis side.
                ConvexCollider {
                    vertices_m: [-config.wheel_radius_m * 0.1, config.wheel_radius_m * 0.1]
                        .into_iter()
                        .flat_map(|z| {
                            (0..32).map(move |i| {
                                let angle = i as f64 * std::f64::consts::TAU / 32.0;
                                Vec3::new(
                                    config.wheel_radius_m * angle.cos(),
                                    config.wheel_radius_m * angle.sin(),
                                    z,
                                )
                            })
                        })
                        .collect(),
                },
                RevoluteJointDesc {
                    parent: base_link,
                    axis: Vec3::NEG_Z,
                    anchor_parent_m: Vec3::new(0.0, wheel_offset.y, z_offset),
                    anchor_child_m: Vec3::ZERO,
                    relative_rotation: Quat::IDENTITY,
                    lower_rad: None,
                    upper_rad: None,
                },
                JointMotor::default(),
                JointMotorGainModel::ForceBased,
            ));
        }

        world.entity_mut(actuator_entity).insert(Actuator {
            robot,
            joint: Some(wheel),
            name: format!("{name}_motor"),
            mode: ControlMode::Velocity,
            target: ActuatorTarget::default(),
            limits: ActuatorLimits {
                max_effort_nm: 1.0,
                min_velocity_rad_s: -config.max_wheel_velocity_rad_s,
                max_velocity_rad_s: config.max_wheel_velocity_rad_s,
                ..ActuatorLimits::default()
            },
        });
    }

    if config.drive_mode == DiffDriveDriveMode::JointDriven {
        // Two driven wheels alone leave pitch unsupported. These low-friction
        // spherical skids keep the chassis clear of the floor without imposing
        // a base pose or velocity; they are fixed supports, not rolling casters.
        let radius_m = config.wheel_radius_m * 0.3;
        for (name, sign) in [("front_caster", 1.0), ("rear_caster", -1.0)] {
            let caster = spawn_named(world, name);
            let offset = Vec3::new(
                sign * config.base_half_extents_m.x * 0.8,
                radius_m - base_translation.y,
                0.0,
            );
            world.entity_mut(caster).insert((
                Link {
                    robot,
                    name: name.into(),
                },
                Transform3::from_translation_rotation(base_translation + offset, Quat::IDENTITY),
                RigidBody {
                    body_type: RigidBodyType::Dynamic,
                    mass_kg: 0.05,
                    ..RigidBody::default()
                },
                Collider {
                    material: PhysicsMaterial {
                        friction: 0.0,
                        restitution: 0.0,
                    },
                    ..Collider::sphere(radius_m)
                },
                FixedJointDesc {
                    parent: base_link,
                    anchor_parent_m: offset,
                    anchor_child_m: Vec3::ZERO,
                    relative_rotation: Quat::IDENTITY,
                },
            ));
        }
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

    #[test]
    fn wheel_geometry_rolls_forward_and_clears_the_chassis() {
        let mut world = World::new();
        let config = DiffDriveConfig {
            drive_mode: DiffDriveDriveMode::JointDriven,
            ..DiffDriveConfig::default()
        };
        let robot = spawn_diff_drive_robot(&mut world, &config);
        let base = world.get::<Transform3>(robot.base_link).unwrap();
        assert!(base.translation.y - config.base_half_extents_m.y > 0.0);
        let left = world.get::<Transform3>(robot.left_wheel).unwrap();
        let right = world.get::<Transform3>(robot.right_wheel).unwrap();
        assert!((right.translation.z - left.translation.z - config.track_width_m).abs() < 1e-12);
        for entity in [robot.left_wheel, robot.right_wheel] {
            let joint = world.get::<RevoluteJointDesc>(entity).unwrap();
            // A positive wheel command drives the ground contact toward -X,
            // so no-slip rolling moves the chassis toward its declared +X.
            let contact_velocity = joint.axis.cross(-Vec3::Y * config.wheel_radius_m);
            assert!((contact_velocity + Vec3::X * config.wheel_radius_m).length() < 1e-12);
            let collider = world.get::<Collider>(entity).unwrap();
            let shape_axis = collider.local_offset.rotation * Vec3::Y;
            assert!(shape_axis.dot(joint.axis).abs() > 1.0 - 1e-12);
            let convex = world.get::<ConvexCollider>(entity).unwrap();
            let wheel = world.get::<Transform3>(entity).unwrap();
            for vertex in &convex.vertices_m {
                // Convex vertices are in link coordinates, independent of the
                // legacy collider's local offset. The entire wheel clears Z.
                assert!((wheel.translation.z + vertex.z).abs() > config.base_half_extents_m.z);
                assert!((vertex.x.hypot(vertex.y) - config.wheel_radius_m).abs() < 1e-12);
                assert!(wheel.translation.y + vertex.y >= -1e-12);
            }
        }
    }
}
