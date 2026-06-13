//! Robot control systems.

use crate::actuator::ControlMode;
use crate::commands::{ActuatorCommand, ActuatorCommandBuffer};
use crate::components::{Actuator, Joint, JointKind};
use crate::diff_drive::DifferentialDrive;
use crate::joint::{validate_joint_position, validate_joint_velocity, JointValidationError};
use bevy_ecs::prelude::{Entity, World};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape, JointMotor, RigidBody, RigidBodyType};
use rne_world::Transform3;

/// Result of applying one actuator command.
#[derive(Clone, Debug, PartialEq)]
pub enum CommandApplyResult {
    /// Command applied successfully.
    Applied,
    /// Command rejected because the target entity was invalid.
    InvalidTarget,
    /// Command rejected because the joint validation failed.
    JointRejected(JointValidationError),
    /// Command ignored because it was stale.
    Stale,
}

/// Applies queued actuator commands to actuators and joints.
pub fn apply_actuator_commands(world: &mut World, buffer: &mut ActuatorCommandBuffer) {
    let entries: Vec<_> = buffer.drain().collect();

    for entry in entries {
        let _ = apply_one_command(world, &entry.command);
    }
}

fn apply_one_command(world: &mut World, command: &ActuatorCommand) -> CommandApplyResult {
    match command {
        ActuatorCommand::JointPosition {
            joint,
            position_rad,
        } => apply_joint_position(world, *joint, *position_rad),
        ActuatorCommand::JointVelocity {
            joint,
            velocity_rad_s,
        } => apply_joint_velocity(world, *joint, *velocity_rad_s),
        ActuatorCommand::JointEffort { joint, effort_nm } => {
            apply_joint_effort(world, *joint, *effort_nm)
        }
        ActuatorCommand::WheelVelocity {
            wheel,
            velocity_rad_s,
        } => apply_wheel_velocity(world, *wheel, *velocity_rad_s),
        ActuatorCommand::GripperWidth { .. } | ActuatorCommand::BodyWrench { .. } => {
            CommandApplyResult::InvalidTarget
        }
        ActuatorCommand::Ackermann { .. } => CommandApplyResult::InvalidTarget,
    }
}

fn apply_joint_position(
    world: &mut World,
    joint_entity: Entity,
    position_rad: f64,
) -> CommandApplyResult {
    let Some(joint) = world.get::<Joint>(joint_entity).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    let validated = match validate_joint_position(&joint, position_rad) {
        Ok(value) => value,
        Err(error) => return CommandApplyResult::JointRejected(error),
    };

    let Some(mut joint_mut) = world.get_mut::<Joint>(joint_entity) else {
        return CommandApplyResult::InvalidTarget;
    };
    joint_mut.position = validated;

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Position;
            actuator.target.position_rad = actuator.limits.clamp_position(validated);
        }
    }

    CommandApplyResult::Applied
}

fn apply_joint_velocity(
    world: &mut World,
    joint_entity: Entity,
    velocity_rad_s: f64,
) -> CommandApplyResult {
    let Some(joint) = world.get::<Joint>(joint_entity).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    if joint.kind == JointKind::Fixed && velocity_rad_s.abs() > f64::EPSILON {
        return CommandApplyResult::JointRejected(JointValidationError::FixedJointNonZero);
    }

    let validated = match validate_joint_velocity(&joint, velocity_rad_s) {
        Ok(value) => value,
        Err(error) => return CommandApplyResult::JointRejected(error),
    };

    if let Some(mut joint_mut) = world.get_mut::<Joint>(joint_entity) {
        joint_mut.velocity = validated;
    }

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Velocity;
            actuator.target.velocity_rad_s = actuator.limits.clamp_velocity(validated);
        }
    }

    CommandApplyResult::Applied
}

fn apply_joint_effort(
    world: &mut World,
    joint_entity: Entity,
    effort_nm: f64,
) -> CommandApplyResult {
    let Some(_joint) = world.get::<Joint>(joint_entity) else {
        return CommandApplyResult::InvalidTarget;
    };

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Effort;
            actuator.target.effort_nm = effort_nm.clamp(
                -actuator.limits.max_effort_nm,
                actuator.limits.max_effort_nm,
            );
            return CommandApplyResult::Applied;
        }
    }

    CommandApplyResult::InvalidTarget
}

fn apply_wheel_velocity(
    world: &mut World,
    wheel_actuator: Entity,
    velocity_rad_s: f64,
) -> CommandApplyResult {
    let Some(actuator) = world.get::<Actuator>(wheel_actuator).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    let clamped = actuator.limits.clamp_velocity(velocity_rad_s);
    let Some(mut actuator_mut) = world.get_mut::<Actuator>(wheel_actuator) else {
        return CommandApplyResult::InvalidTarget;
    };
    actuator_mut.mode = ControlMode::Velocity;
    actuator_mut.target.velocity_rad_s = clamped;

    if let Some(joint_entity) = actuator_mut.joint {
        if let Some(mut joint) = world.get_mut::<Joint>(joint_entity) {
            joint.velocity = clamped;
        }
    }

    CommandApplyResult::Applied
}

fn find_actuator_for_joint(world: &World, joint_entity: Entity) -> Option<Entity> {
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world
            .get::<Actuator>(entity)
            .is_some_and(|actuator| actuator.joint == Some(joint_entity))
        {
            return Some(entity);
        }
    }
    None
}

/// Integrates differential drive kinematics for one simulation step.
pub fn differential_drive_kinematics(
    world: &mut World,
    drives: &[DifferentialDrive],
    dt: SimDuration,
) {
    let dt_s = dt.as_seconds().value();

    for drive in drives {
        let Some(left) = world.get::<Actuator>(drive.left_actuator) else {
            continue;
        };
        let Some(right) = world.get::<Actuator>(drive.right_actuator) else {
            continue;
        };

        let v_left = left.target.velocity_rad_s * drive.wheel_radius_m;
        let v_right = right.target.velocity_rad_s * drive.wheel_radius_m;
        let linear_m_s = (v_left + v_right) * 0.5;
        let yaw_rad_s = (v_right - v_left) / drive.track_width_m;

        let (base_snapshot, forward) = {
            let Some(mut transform) = world.get_mut::<Transform3>(drive.base_link) else {
                continue;
            };

            let forward = transform.rotation * Vec3::X;
            transform.translation += forward * linear_m_s * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_rad_s * dt_s) * transform.rotation).normalize();
            (*transform, forward)
        };

        if world
            .get::<RigidBody>(drive.base_link)
            .is_some_and(|body| body.body_type == RigidBodyType::Kinematic)
        {
            sync_wheel_transforms(world, drive, &base_snapshot);
        }

        if let Some(mut body) = world.get_mut::<RigidBody>(drive.base_link) {
            let forward_flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
            body.linear_velocity_m_s = forward_flat * linear_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rad_s, 0.0);
        }
    }
}

fn sync_wheel_transforms(world: &mut World, drive: &DifferentialDrive, base: &Transform3) {
    let half_track = drive.track_width_m * 0.5;
    let wheel_y = world
        .get::<Collider>(drive.base_link)
        .and_then(|collider| match collider.shape {
            ColliderShape::Cuboid { half_extents_m } => {
                Some(-half_extents_m.y + drive.wheel_radius_m)
            }
            _ => None,
        })
        .unwrap_or(0.0);

    for (wheel, x_offset) in [
        (drive.left_actuator, -half_track),
        (drive.right_actuator, half_track),
    ] {
        let Some(actuator) = world.get::<Actuator>(wheel) else {
            continue;
        };
        let Some(wheel_entity) = actuator.joint else {
            continue;
        };
        let Some(mut wheel_transform) = world.get_mut::<Transform3>(wheel_entity) else {
            continue;
        };
        let offset = base.rotation * Vec3::new(x_offset, wheel_y, 0.0);
        wheel_transform.translation = base.translation + offset;
        wheel_transform.rotation = base.rotation;
    }
}

/// Copies actuator velocity targets into [`JointMotor`] components for physics stepping.
pub fn sync_joint_motors_from_actuators(world: &mut World, drives: &[DifferentialDrive]) {
    for drive in drives {
        for actuator_entity in [drive.left_actuator, drive.right_actuator] {
            let Some(actuator) = world.get::<Actuator>(actuator_entity) else {
                continue;
            };
            let Some(joint_entity) = actuator.joint else {
                continue;
            };
            let velocity = actuator.target.velocity_rad_s;
            if let Some(mut motor) = world.get_mut::<JointMotor>(joint_entity) {
                motor.velocity_rad_s = velocity;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuator::ActuatorLimits;
    use crate::components::{JointKind, JointLimits, Link, Robot, RobotId};
    use rne_core::SimTime;
    use rne_ecs::spawn_named;
    use rne_math::Seconds;

    fn setup_robot_with_joint() -> (World, Entity, Entity, Entity) {
        let mut world = World::new();
        let robot_entity = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let wheel = spawn_named(&mut world, "wheel");

        world.entity_mut(robot_entity).insert(Robot {
            robot_id: RobotId::default(),
            model_name: "test".into(),
            base_link: base,
        });
        world.entity_mut(base).insert(Link {
            robot: robot_entity,
            name: "base".into(),
        });
        world.entity_mut(wheel).insert((
            Link {
                robot: robot_entity,
                name: "wheel".into(),
            },
            Joint {
                robot: robot_entity,
                parent_link: base,
                child_link: wheel,
                kind: JointKind::Continuous,
                limits: JointLimits::default(),
                axis: Vec3::Y,
                position: 0.0,
                velocity: 0.0,
            },
            Actuator {
                robot: robot_entity,
                joint: Some(wheel),
                name: "wheel_motor".into(),
                mode: ControlMode::Velocity,
                target: Default::default(),
                limits: ActuatorLimits::default(),
            },
        ));

        (world, robot_entity, wheel, wheel)
    }

    #[test]
    fn valid_command_applies() {
        let (mut world, _, joint, actuator) = setup_robot_with_joint();
        let mut buffer = ActuatorCommandBuffer::new();
        buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: actuator,
                velocity_rad_s: 3.0,
            },
            SimTime::ZERO,
        );
        apply_actuator_commands(&mut world, &mut buffer);
        assert_eq!(
            world
                .get::<Actuator>(actuator)
                .unwrap()
                .target
                .velocity_rad_s,
            3.0
        );
        assert_eq!(world.get::<Joint>(joint).unwrap().velocity, 3.0);
    }

    #[test]
    fn invalid_joint_command_rejected() {
        let (mut world, _, joint, _) = setup_robot_with_joint();
        world.get_mut::<Joint>(joint).unwrap().kind = JointKind::Fixed;
        let result = apply_joint_velocity(&mut world, joint, 1.0);
        assert!(matches!(
            result,
            CommandApplyResult::JointRejected(JointValidationError::FixedJointNonZero)
        ));
    }

    #[test]
    fn diff_drive_moves_forward() {
        let mut world = World::new();
        let spawned = crate::diff_drive::spawn_diff_drive_robot(
            &mut world,
            &crate::diff_drive::DiffDriveConfig::default(),
        );

        let mut buffer = ActuatorCommandBuffer::new();
        buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: spawned.left_actuator,
                velocity_rad_s: 5.0,
            },
            SimTime::ZERO,
        );
        buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: spawned.right_actuator,
                velocity_rad_s: 5.0,
            },
            SimTime::ZERO,
        );
        apply_actuator_commands(&mut world, &mut buffer);

        differential_drive_kinematics(
            &mut world,
            &[spawned.drive],
            SimDuration::from_seconds(Seconds::new(1.0)),
        );

        let x = world
            .get::<Transform3>(spawned.base_link)
            .unwrap()
            .translation
            .x;
        assert!(x > 0.0, "robot should move forward, x={x}");
    }
}
