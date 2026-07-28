//! Robot control systems.

use crate::actuator::ControlMode;
use crate::commands::{ActuatorCommand, ActuatorCommandBuffer};
use crate::components::{AckermannDrive, Actuator, Joint, JointKind};
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

/// Result of commanding a kinematic Ackermann drive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AckermannCommandResult {
    /// The finite command was clamped to the drive limits and applied.
    Applied,
    /// The target entity has no valid [`AckermannDrive`].
    InvalidTarget,
    /// At least one command value was non-finite; the previous target was preserved.
    NonFiniteCommand,
}

/// Applies a bounded speed and steering target to one kinematic Ackermann vehicle.
pub fn command_ackermann_drive(
    world: &mut World,
    vehicle: Entity,
    speed_m_s: f64,
    steering_rad: f64,
) -> AckermannCommandResult {
    if !speed_m_s.is_finite() || !steering_rad.is_finite() {
        return AckermannCommandResult::NonFiniteCommand;
    }
    let Some(mut drive) = world.get_mut::<AckermannDrive>(vehicle) else {
        return AckermannCommandResult::InvalidTarget;
    };
    if !drive.is_valid() {
        return AckermannCommandResult::InvalidTarget;
    }
    drive.target_speed_m_s = speed_m_s.clamp(-drive.max_speed_m_s, drive.max_speed_m_s);
    drive.target_steering_rad = steering_rad.clamp(-drive.max_steering_rad, drive.max_steering_rad);
    AckermannCommandResult::Applied
}

/// Integrates every valid Ackermann vehicle in stable entity order for one fixed step.
///
/// Invalid drive configurations and entities without a [`Transform3`] are left unchanged.
pub fn ackermann_kinematics(world: &mut World, dt: SimDuration) {
    let dt_s = dt.as_seconds().value();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return;
    }
    let mut vehicles: Vec<Entity> = world
        .iter_entities()
        .filter(|entity| entity.contains::<AckermannDrive>() && entity.contains::<Transform3>())
        .map(|entity| entity.id())
        .collect();
    vehicles.sort_by_key(|entity| entity.to_bits());

    for vehicle in vehicles {
        let Some(mut drive) = world.get::<AckermannDrive>(vehicle).cloned() else {
            continue;
        };
        if !drive.is_valid() {
            continue;
        }
        let accelerating = drive.target_speed_m_s.signum() == drive.speed_m_s.signum()
            && drive.target_speed_m_s.abs() > drive.speed_m_s.abs();
        let speed_rate_m_s2 = if accelerating {
            drive.max_acceleration_m_s2
        } else {
            drive.max_deceleration_m_s2
        };
        drive.speed_m_s = move_towards(
            drive.speed_m_s,
            drive.target_speed_m_s,
            speed_rate_m_s2 * dt_s,
        );
        drive.steering_rad = move_towards(
            drive.steering_rad,
            drive.target_steering_rad,
            drive.max_steering_rate_rad_s * dt_s,
        );
        let yaw_rad_s = drive.speed_m_s / drive.wheelbase_m * drive.steering_rad.tan();
        let yaw_delta_rad = yaw_rad_s * dt_s;
        let mut forward = Vec3::X;
        if let Some(mut transform) = world.get_mut::<Transform3>(vehicle) {
            let midpoint_rotation =
                (Quat::from_rotation_y(yaw_delta_rad * 0.5) * transform.rotation).normalize();
            forward = midpoint_rotation * Vec3::X;
            transform.translation += forward * drive.speed_m_s * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_delta_rad) * transform.rotation).normalize();
        }
        if let Some(mut body) = world.get_mut::<RigidBody>(vehicle) {
            body.linear_velocity_m_s = forward * drive.speed_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rad_s, 0.0);
        }
        world.entity_mut(vehicle).insert(drive);
    }
}

/// Computes a pure-pursuit steering target toward a world-space lookahead point.
///
/// The returned angle follows the Ackermann convention used by
/// [`ackermann_kinematics`] and is not clamped to a particular vehicle's limits.
pub fn pure_pursuit_steering(
    transform: &Transform3,
    target_m: Vec3,
    wheelbase_m: f64,
    lookahead_m: f64,
) -> f64 {
    if !wheelbase_m.is_finite()
        || !lookahead_m.is_finite()
        || wheelbase_m <= 0.0
        || lookahead_m <= 0.0
    {
        return 0.0;
    }
    let local_target = transform.rotation.conjugate() * (target_m - transform.translation);
    (-2.0 * wheelbase_m * local_target.z).atan2(lookahead_m * lookahead_m)
}

fn move_towards(current: f64, target: f64, max_delta: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
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
        ActuatorCommand::Ackermann {
            vehicle,
            speed_m_s,
            steering_rad,
        } => match command_ackermann_drive(world, *vehicle, *speed_m_s, *steering_rad) {
            AckermannCommandResult::Applied => CommandApplyResult::Applied,
            AckermannCommandResult::InvalidTarget | AckermannCommandResult::NonFiniteCommand => {
                CommandApplyResult::InvalidTarget
            }
        },
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
    use crate::components::{AckermannDrive, JointKind, JointLimits, Link, Robot, RobotId};
    use rne_core::{SimClock, SimTime};
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

    #[test]
    fn ackermann_commands_clamp_and_integrate_from_sim_clock() {
        let mut world = World::new();
        let vehicle = spawn_named(&mut world, "test_vehicle");
        world
            .entity_mut(vehicle)
            .insert((Transform3::default(), AckermannDrive::default()));
        assert_eq!(
            command_ackermann_drive(&mut world, vehicle, 100.0, 2.0),
            AckermannCommandResult::Applied
        );
        let commanded = world.get::<AckermannDrive>(vehicle).unwrap();
        assert_eq!(commanded.target_speed_m_s, commanded.max_speed_m_s);
        assert_eq!(commanded.target_steering_rad, commanded.max_steering_rad);

        let fixed_delta = SimDuration::from_seconds(Seconds::new(1.0 / 60.0));
        let mut clock = SimClock::new(fixed_delta);
        for _ in 0..60 {
            assert_eq!(clock.advance(fixed_delta), 1);
            ackermann_kinematics(&mut world, clock.fixed_delta());
        }
        let transform = world.get::<Transform3>(vehicle).unwrap();
        let drive = world.get::<AckermannDrive>(vehicle).unwrap();
        assert!(drive.speed_m_s > 2.4 && drive.speed_m_s < 2.6);
        assert!(transform.translation.length() > 1.0);
        assert_eq!(clock.sim_time().ticks(), fixed_delta.ticks() * 60);
    }

    #[test]
    fn ackermann_rejects_non_finite_command_without_mutation() {
        let mut world = World::new();
        let vehicle = spawn_named(&mut world, "test_vehicle");
        world
            .entity_mut(vehicle)
            .insert((Transform3::default(), AckermannDrive::default()));
        let before = world.get::<AckermannDrive>(vehicle).unwrap().clone();
        assert_eq!(
            command_ackermann_drive(&mut world, vehicle, f64::NAN, 0.0),
            AckermannCommandResult::NonFiniteCommand
        );
        assert_eq!(world.get::<AckermannDrive>(vehicle).unwrap(), &before);
    }

    #[test]
    fn pure_pursuit_steers_toward_lateral_target() {
        let transform = Transform3::default();
        let steering = pure_pursuit_steering(&transform, Vec3::new(5.0, 0.0, 2.0), 2.7, 5.0);
        assert!(steering < 0.0);
    }
}
