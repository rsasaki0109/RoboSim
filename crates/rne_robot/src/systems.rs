//! Robot control systems.

use crate::actuator::ControlMode;
use crate::commands::{ActuatorCommand, ActuatorCommandBuffer};
use crate::components::{
    AckermannDrive, Actuator, Joint, JointKind, MultirotorFlight, VehicleDynamics,
};
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

/// Result of commanding a multirotor position target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultirotorCommandResult {
    /// The finite position and heading target was applied.
    Applied,
    /// The target entity has no valid [`MultirotorFlight`].
    InvalidTarget,
    /// At least one command value was non-finite; the previous target was preserved.
    NonFiniteCommand,
}

/// Applies a world-space position and heading target to one multirotor.
///
/// Commands are accepted only when both the target and the existing flight
/// component are valid. Rejected commands leave the previous target unchanged.
pub fn command_multirotor(
    world: &mut World,
    aircraft: Entity,
    target_position_m: Vec3,
    target_yaw_rad: f64,
) -> MultirotorCommandResult {
    if !target_position_m.is_finite() || !target_yaw_rad.is_finite() {
        return MultirotorCommandResult::NonFiniteCommand;
    }
    let Some(mut flight) = world.get_mut::<MultirotorFlight>(aircraft) else {
        return MultirotorCommandResult::InvalidTarget;
    };
    if !flight.is_valid() {
        return MultirotorCommandResult::InvalidTarget;
    }
    flight.target_position_m = target_position_m;
    flight.target_yaw_rad = wrap_angle_rad(target_yaw_rad);
    MultirotorCommandResult::Applied
}

/// Advances every valid multirotor in stable entity order for one fixed step.
///
/// The deterministic cascade is position error to desired velocity, desired
/// velocity to bounded acceleration, then semi-implicit position integration.
/// A Y-up body attitude follows the required thrust direction without exceeding
/// [`MultirotorFlight::max_tilt_rad`]. Entities with invalid configurations or
/// without a [`Transform3`] are left unchanged.
pub fn multirotor_flight(world: &mut World, dt: SimDuration) {
    let dt_s = dt.as_seconds().value();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return;
    }
    let mut aircraft: Vec<Entity> = world
        .iter_entities()
        .filter(|entity| entity.contains::<MultirotorFlight>() && entity.contains::<Transform3>())
        .map(|entity| entity.id())
        .collect();
    aircraft.sort_by_key(|entity| entity.to_bits());

    for entity in aircraft {
        let Some(mut flight) = world.get::<MultirotorFlight>(entity).copied() else {
            continue;
        };
        let Some(mut transform) = world.get::<Transform3>(entity).copied() else {
            continue;
        };
        if !flight.is_valid()
            || !transform.translation.is_finite()
            || !transform.rotation.is_finite()
        {
            continue;
        }

        let position_error_m = flight.target_position_m - transform.translation;
        let mut desired_velocity_m_s = position_error_m * flight.position_gain_s_inv;
        desired_velocity_m_s.y = desired_velocity_m_s
            .y
            .clamp(-flight.max_climb_speed_m_s, flight.max_climb_speed_m_s);
        let horizontal_speed_m_s = desired_velocity_m_s.x.hypot(desired_velocity_m_s.z);
        if horizontal_speed_m_s > flight.max_horizontal_speed_m_s {
            let scale = flight.max_horizontal_speed_m_s / horizontal_speed_m_s;
            desired_velocity_m_s.x *= scale;
            desired_velocity_m_s.z *= scale;
        }

        let mut acceleration_m_s2 =
            (desired_velocity_m_s - flight.velocity_m_s) * flight.velocity_gain_s_inv;
        acceleration_m_s2 = clamp_length(acceleration_m_s2, flight.max_acceleration_m_s2);
        let horizontal_tilt_limit_m_s2 = 9.81 * flight.max_tilt_rad.tan();
        let horizontal_acceleration_m_s2 = acceleration_m_s2.x.hypot(acceleration_m_s2.z);
        if horizontal_acceleration_m_s2 > horizontal_tilt_limit_m_s2 {
            let scale = horizontal_tilt_limit_m_s2 / horizontal_acceleration_m_s2;
            acceleration_m_s2.x *= scale;
            acceleration_m_s2.z *= scale;
        }

        flight.velocity_m_s += acceleration_m_s2 * dt_s;
        flight.velocity_m_s.y = flight
            .velocity_m_s
            .y
            .clamp(-flight.max_climb_speed_m_s, flight.max_climb_speed_m_s);
        let horizontal_velocity_m_s = flight.velocity_m_s.x.hypot(flight.velocity_m_s.z);
        if horizontal_velocity_m_s > flight.max_horizontal_speed_m_s {
            let scale = flight.max_horizontal_speed_m_s / horizontal_velocity_m_s;
            flight.velocity_m_s.x *= scale;
            flight.velocity_m_s.z *= scale;
        }
        transform.translation += flight.velocity_m_s * dt_s;

        let yaw_error_rad = wrap_angle_rad(flight.target_yaw_rad - flight.yaw_rad);
        let yaw_rate_rad_s =
            (yaw_error_rad * 3.0).clamp(-flight.max_yaw_rate_rad_s, flight.max_yaw_rate_rad_s);
        flight.yaw_rad = wrap_angle_rad(flight.yaw_rad + yaw_rate_rad_s * dt_s);

        let horizontal_acceleration = Vec3::new(acceleration_m_s2.x, 0.0, acceleration_m_s2.z);
        let desired_up = (Vec3::Y + horizontal_acceleration / 9.81).normalize_or_zero();
        let tilt = Quat::from_rotation_arc(Vec3::Y, desired_up);
        let yaw = Quat::from_rotation_y(flight.yaw_rad);
        let desired_rotation = (tilt * yaw).normalize();
        let attitude_blend = if flight.attitude_response_s == 0.0 {
            1.0
        } else {
            1.0 - (-dt_s / flight.attitude_response_s).exp()
        };
        transform.rotation = transform
            .rotation
            .slerp(desired_rotation, attitude_blend)
            .normalize();

        flight.commanded_acceleration_m_s2 = acceleration_m_s2;
        if let Some(mut body) = world.get_mut::<RigidBody>(entity) {
            body.linear_velocity_m_s = flight.velocity_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rate_rad_s, 0.0);
        }
        world.entity_mut(entity).insert((flight, transform));
    }
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
        .filter(|entity| {
            entity.contains::<AckermannDrive>()
                && entity.contains::<Transform3>()
                // Vehicles carrying VehicleDynamics are integrated by the dynamic
                // model instead; running both would double-integrate the chassis.
                && !entity.contains::<VehicleDynamics>()
        })
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

/// Advances vehicles that carry both [`AckermannDrive`] and [`VehicleDynamics`] with a
/// planar dynamic bicycle model.
///
/// [`ackermann_kinematics`] must not also run over these vehicles; this system is the
/// dynamic replacement, not a correction pass. Command shaping (speed and steering rate
/// limits) is shared with the kinematic path so the two models receive identical inputs
/// and differ only in how the chassis answers them.
///
/// Per step, for forward speed `vx`, lateral speed `vy`, yaw rate `r`, steering `delta`,
/// axle distances `a`/`b`, and per-axle cornering stiffness `C`:
///
/// ```text
/// alpha_f = atan((vy + a r) / vx) - delta      front slip angle
/// alpha_r = atan((vy - b r) / vx)              rear slip angle
/// Fy      = clamp(-C alpha, +/- mu Fz)         linear tire, friction saturated
/// m (vy' + vx r) = Fyf cos(delta) + Fyr        lateral balance
/// Iz r'          = a Fyf cos(delta) - b Fyr    yaw balance
/// ```
///
/// `Fz` per axle includes longitudinal load transfer `m ax h / L`, so braking loads the
/// front tires and throttle loads the rear — which is why the same corner behaves
/// differently on and off the power. Below [`VehicleDynamics::blend_low_speed_m_s`] the
/// lateral states relax toward the kinematic solution to avoid the `1/vx` singularity.
pub fn vehicle_dynamics(world: &mut World, dt: SimDuration) {
    let dt_s = dt.as_seconds().value();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return;
    }
    let mut vehicles: Vec<Entity> = world
        .iter_entities()
        .filter(|entity| {
            entity.contains::<AckermannDrive>()
                && entity.contains::<VehicleDynamics>()
                && entity.contains::<Transform3>()
        })
        .map(|entity| entity.id())
        .collect();
    vehicles.sort_by_key(|entity| entity.to_bits());

    for vehicle in vehicles {
        let Some(mut drive) = world.get::<AckermannDrive>(vehicle).cloned() else {
            continue;
        };
        let Some(mut dynamics) = world.get::<VehicleDynamics>(vehicle).copied() else {
            continue;
        };
        if !drive.is_valid() || !dynamics.is_valid() {
            continue;
        }

        // Shared command shaping, identical to the kinematic path.
        let accelerating = drive.target_speed_m_s.signum() == drive.speed_m_s.signum()
            && drive.target_speed_m_s.abs() > drive.speed_m_s.abs();
        let speed_rate_m_s2 = if accelerating {
            drive.max_acceleration_m_s2
        } else {
            drive.max_deceleration_m_s2
        };
        let previous_speed_m_s = drive.speed_m_s;
        drive.speed_m_s = move_towards(
            drive.speed_m_s,
            drive.target_speed_m_s,
            speed_rate_m_s2 * dt_s,
        );
        // Steering passes through the first-order actuator lag before the rate limit.
        // With a zero time constant the lag target is the command itself and this
        // reduces exactly to the kinematic path's shaping.
        let lag_target = if dynamics.steering_lag_s > 0.0 {
            let alpha = 1.0 - (-dt_s / dynamics.steering_lag_s).exp();
            drive.steering_rad + (drive.target_steering_rad - drive.steering_rad) * alpha
        } else {
            drive.target_steering_rad
        };
        drive.steering_rad = move_towards(
            drive.steering_rad,
            lag_target,
            drive.max_steering_rate_rad_s * dt_s,
        );

        let vx = drive.speed_m_s;
        let ax = (drive.speed_m_s - previous_speed_m_s) / dt_s;
        let delta = drive.steering_rad;
        let wheelbase = dynamics.wheelbase_m();

        // Axle loads with longitudinal transfer; clamped so neither axle lifts.
        let transfer_n = dynamics.mass_kg * ax * dynamics.center_of_mass_height_m / wheelbase;
        let front_load_n = (dynamics.static_front_load_n() - transfer_n).max(0.0);
        let rear_load_n = (dynamics.static_rear_load_n() + transfer_n).max(0.0);

        let kinematic_yaw_rate = vx / wheelbase * delta.tan();
        let speed_abs = vx.abs();

        if speed_abs <= dynamics.blend_low_speed_m_s.max(f64::EPSILON) {
            // Kinematic regime: slip angles are undefined, so the lateral states take
            // the no-slip solution directly.
            dynamics.yaw_rate_rad_s = kinematic_yaw_rate;
            dynamics.lateral_velocity_m_s = kinematic_yaw_rate * dynamics.rear_axle_m;
            dynamics.front_slip_rad = 0.0;
            dynamics.rear_slip_rad = 0.0;
            dynamics.front_saturated = false;
            dynamics.rear_saturated = false;
        } else {
            let vy = dynamics.lateral_velocity_m_s;
            let r = dynamics.yaw_rate_rad_s;

            let alpha_f = ((vy + dynamics.front_axle_m * r) / vx).atan() - delta;
            let alpha_r = ((vy - dynamics.rear_axle_m * r) / vx).atan();

            let front_limit_n = dynamics.friction_coefficient * front_load_n;
            let rear_limit_n = dynamics.friction_coefficient * rear_load_n;
            let front_force_n = (-dynamics.front_cornering_stiffness_n_rad * alpha_f)
                .clamp(-front_limit_n, front_limit_n);
            let rear_force_n = (-dynamics.rear_cornering_stiffness_n_rad * alpha_r)
                .clamp(-rear_limit_n, rear_limit_n);

            dynamics.front_slip_rad = alpha_f;
            dynamics.rear_slip_rad = alpha_r;
            dynamics.front_saturated =
                (dynamics.front_cornering_stiffness_n_rad * alpha_f).abs() > front_limit_n;
            dynamics.rear_saturated =
                (dynamics.rear_cornering_stiffness_n_rad * alpha_r).abs() > rear_limit_n;

            let lateral_acceleration =
                (front_force_n * delta.cos() + rear_force_n) / dynamics.mass_kg - vx * r;
            let yaw_acceleration = (dynamics.front_axle_m * front_force_n * delta.cos()
                - dynamics.rear_axle_m * rear_force_n)
                / dynamics.yaw_inertia_kg_m2;

            dynamics.lateral_velocity_m_s += lateral_acceleration * dt_s;
            dynamics.yaw_rate_rad_s += yaw_acceleration * dt_s;
        }

        let yaw_delta_rad = dynamics.yaw_rate_rad_s * dt_s;
        let mut velocity_world = Vec3::ZERO;
        if let Some(mut transform) = world.get_mut::<Transform3>(vehicle) {
            let midpoint_rotation =
                (Quat::from_rotation_y(yaw_delta_rad * 0.5) * transform.rotation).normalize();
            // The body carries both forward and lateral velocity; slip is precisely
            // the difference between where the nose points and where the car goes.
            velocity_world = midpoint_rotation * Vec3::new(vx, 0.0, -dynamics.lateral_velocity_m_s);
            transform.translation += velocity_world * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_delta_rad) * transform.rotation).normalize();
        }
        if let Some(mut body) = world.get_mut::<RigidBody>(vehicle) {
            body.linear_velocity_m_s = velocity_world;
            body.angular_velocity_rad_s = Vec3::new(0.0, dynamics.yaw_rate_rad_s, 0.0);
        }
        world.entity_mut(vehicle).insert((drive, dynamics));
    }
}

fn move_towards(current: f64, target: f64, max_delta: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

fn clamp_length(value: Vec3, max_length: f64) -> Vec3 {
    let length = value.length();
    if length > max_length && length > 0.0 {
        value * (max_length / length)
    } else {
        value
    }
}

fn wrap_angle_rad(mut angle_rad: f64) -> f64 {
    while angle_rad > std::f64::consts::PI {
        angle_rad -= std::f64::consts::TAU;
    }
    while angle_rad < -std::f64::consts::PI {
        angle_rad += std::f64::consts::TAU;
    }
    angle_rad
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

/// Copies every actuator's backend-neutral target into its linked [`JointMotor`].
///
/// The optional `drives` argument on [`sync_joint_motors_from_actuators`] is kept
/// for source compatibility with older diff-drive callers. Named URDF actuators
/// use this function directly and are resolved through their [`Joint`] child link.
pub fn sync_all_joint_motors_from_actuators(world: &mut World) {
    let mut actuator_entities: Vec<_> = world
        .iter_entities()
        .map(|entity| entity.id())
        .filter(|entity| world.get::<Actuator>(*entity).is_some())
        .collect();
    actuator_entities.sort_unstable();

    for actuator_entity in actuator_entities {
        let Some((joint_entity, mode, target)) = world
            .get::<Actuator>(actuator_entity)
            .map(|actuator| (actuator.joint, actuator.mode, actuator.target))
        else {
            continue;
        };
        let Some(joint_entity) = joint_entity else {
            continue;
        };
        let Some(child_link) = world
            .get::<Joint>(joint_entity)
            .map(|joint| joint.child_link)
        else {
            continue;
        };
        let Some(mut motor) = world.get_mut::<JointMotor>(child_link) else {
            continue;
        };
        motor.velocity_rad_s = match mode {
            ControlMode::Velocity => target.velocity_rad_s,
            ControlMode::Position | ControlMode::Effort => 0.0,
        };
    }
}

/// Copies actuator velocity targets into [`JointMotor`] components for physics stepping.
pub fn sync_joint_motors_from_actuators(world: &mut World, _drives: &[DifferentialDrive]) {
    sync_all_joint_motors_from_actuators(world);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuator::ActuatorLimits;
    use crate::components::{
        AckermannDrive, JointKind, JointLimits, Link, MultirotorFlight, Robot, RobotId,
    };
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

    fn run_multirotor_replay() -> (Transform3, MultirotorFlight, f64, f64, f64, f64) {
        let mut world = World::new();
        let aircraft = spawn_named(&mut world, "showcase_uav");
        world.entity_mut(aircraft).insert((
            Transform3 {
                translation: Vec3::new(-18.0, 8.0, 12.0),
                ..Transform3::IDENTITY
            },
            MultirotorFlight::default(),
            RigidBody::default(),
        ));
        assert_eq!(
            command_multirotor(&mut world, aircraft, Vec3::new(22.0, 14.0, -16.0), 1.1,),
            MultirotorCommandResult::Applied
        );

        let dt = SimDuration::from_seconds(Seconds::new(1.0 / 60.0));
        let mut maximum_speed_m_s: f64 = 0.0;
        let mut maximum_acceleration_m_s2: f64 = 0.0;
        let mut maximum_tilt_rad: f64 = 0.0;
        let mut maximum_yaw_rate_rad_s: f64 = 0.0;
        for _ in 0..720 {
            multirotor_flight(&mut world, dt);
            let flight = world.get::<MultirotorFlight>(aircraft).unwrap();
            let transform = world.get::<Transform3>(aircraft).unwrap();
            maximum_speed_m_s = maximum_speed_m_s.max(flight.velocity_m_s.length());
            maximum_acceleration_m_s2 =
                maximum_acceleration_m_s2.max(flight.commanded_acceleration_m_s2.length());
            let body_up = transform.rotation * Vec3::Y;
            maximum_tilt_rad = maximum_tilt_rad.max(body_up.dot(Vec3::Y).clamp(-1.0, 1.0).acos());
            maximum_yaw_rate_rad_s = maximum_yaw_rate_rad_s.max(
                world
                    .get::<RigidBody>(aircraft)
                    .unwrap()
                    .angular_velocity_rad_s
                    .y
                    .abs(),
            );
        }
        (
            *world.get::<Transform3>(aircraft).unwrap(),
            *world.get::<MultirotorFlight>(aircraft).unwrap(),
            maximum_speed_m_s,
            maximum_acceleration_m_s2,
            maximum_tilt_rad,
            maximum_yaw_rate_rad_s,
        )
    }

    #[test]
    fn multirotor_tracks_target_with_bounded_flight_state() {
        let (
            transform,
            flight,
            maximum_speed_m_s,
            maximum_acceleration_m_s2,
            maximum_tilt_rad,
            maximum_yaw_rate_rad_s,
        ) = run_multirotor_replay();
        let error_m = (transform.translation - flight.target_position_m).length();
        assert!(error_m < 0.15, "position error was {error_m:.3} m");
        assert!(
            maximum_speed_m_s
                <= flight
                    .max_horizontal_speed_m_s
                    .hypot(flight.max_climb_speed_m_s)
                    + 1.0e-9
        );
        assert!(maximum_acceleration_m_s2 <= flight.max_acceleration_m_s2 + 1.0e-9);
        assert!(maximum_tilt_rad <= flight.max_tilt_rad + 1.0e-6);
        assert!(maximum_yaw_rate_rad_s <= flight.max_yaw_rate_rad_s + 1.0e-9);
        assert!(wrap_angle_rad(flight.yaw_rad - flight.target_yaw_rad).abs() < 1.0e-6);
    }

    #[test]
    fn multirotor_replay_is_exactly_deterministic() {
        assert_eq!(run_multirotor_replay(), run_multirotor_replay());
    }

    #[test]
    fn multirotor_rejects_non_finite_command_without_mutation() {
        let mut world = World::new();
        let aircraft = spawn_named(&mut world, "showcase_uav");
        world
            .entity_mut(aircraft)
            .insert((Transform3::IDENTITY, MultirotorFlight::default()));
        let before = *world.get::<MultirotorFlight>(aircraft).unwrap();
        assert_eq!(
            command_multirotor(&mut world, aircraft, Vec3::new(f64::NAN, 2.0, 3.0), 0.0),
            MultirotorCommandResult::NonFiniteCommand
        );
        assert_eq!(*world.get::<MultirotorFlight>(aircraft).unwrap(), before);
    }

    #[test]
    fn invalid_multirotor_configuration_is_transactional() {
        let mut world = World::new();
        let aircraft = spawn_named(&mut world, "showcase_uav");
        let flight = MultirotorFlight {
            max_tilt_rad: std::f64::consts::PI,
            ..MultirotorFlight::default()
        };
        let transform = Transform3 {
            translation: Vec3::new(1.0, 2.0, 3.0),
            ..Transform3::IDENTITY
        };
        world.entity_mut(aircraft).insert((transform, flight));
        multirotor_flight(
            &mut world,
            SimDuration::from_seconds(Seconds::new(1.0 / 60.0)),
        );
        assert_eq!(*world.get::<Transform3>(aircraft).unwrap(), transform);
        assert_eq!(*world.get::<MultirotorFlight>(aircraft).unwrap(), flight);
    }

    #[test]
    fn pure_pursuit_steers_toward_lateral_target() {
        let transform = Transform3::default();
        let steering = pure_pursuit_steering(&transform, Vec3::new(5.0, 0.0, 2.0), 2.7, 5.0);
        assert!(steering < 0.0);
    }

    fn spawn_dynamic_vehicle(
        world: &mut World,
        drive: AckermannDrive,
        dynamics: VehicleDynamics,
    ) -> Entity {
        let vehicle = world.spawn_empty().id();
        world.entity_mut(vehicle).insert((
            drive,
            dynamics,
            Transform3::IDENTITY,
            RigidBody::default(),
        ));
        vehicle
    }

    fn hot_lap_drive(speed_m_s: f64, steering_rad: f64) -> AckermannDrive {
        AckermannDrive {
            max_speed_m_s: 60.0,
            max_acceleration_m_s2: 1_000.0,
            max_deceleration_m_s2: 1_000.0,
            max_steering_rate_rad_s: 1_000.0,
            speed_m_s,
            target_speed_m_s: speed_m_s,
            steering_rad,
            target_steering_rad: steering_rad,
            ..AckermannDrive::default()
        }
    }

    fn step_seconds(world: &mut World, seconds: f64) {
        let dt = SimDuration::from_seconds(rne_math::Seconds::new(1.0 / 240.0));
        for _ in 0..(seconds * 240.0) as usize {
            vehicle_dynamics(world, dt);
        }
    }

    #[test]
    fn dynamic_model_matches_kinematics_at_low_speed() {
        // 1.5 m/s is inside the blend region, so the no-slip solution applies.
        let speed = 1.5;
        let steering = 0.3;

        let mut dynamic_world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut dynamic_world,
            hot_lap_drive(speed, steering),
            VehicleDynamics::default(),
        );
        step_seconds(&mut dynamic_world, 2.0);

        let mut kinematic_world = World::new();
        let reference = kinematic_world.spawn_empty().id();
        kinematic_world.entity_mut(reference).insert((
            hot_lap_drive(speed, steering),
            Transform3::IDENTITY,
            RigidBody::default(),
        ));
        let dt = SimDuration::from_seconds(rne_math::Seconds::new(1.0 / 240.0));
        for _ in 0..480 {
            ackermann_kinematics(&mut kinematic_world, dt);
        }

        let dynamic_transform = *dynamic_world.get::<Transform3>(vehicle).unwrap();
        let kinematic_transform = *kinematic_world.get::<Transform3>(reference).unwrap();

        // Headings must agree: the blend takes the no-slip yaw rate exactly.
        let dynamic_forward = dynamic_transform.rotation * Vec3::X;
        let kinematic_forward = kinematic_transform.rotation * Vec3::X;
        assert!(dynamic_forward.dot(kinematic_forward) > 0.999_999);

        // The two models track different chassis points — the dynamic model follows the
        // center of mass, the kinematic one its reference axle — so their paths differ
        // laterally by at most the CG offset times the accumulated yaw.
        let total_yaw = 1.5 / VehicleDynamics::default().wheelbase_m() * 0.3_f64.tan() * 2.0;
        let bound = VehicleDynamics::default().rear_axle_m * total_yaw + 0.05;
        let divergence = (dynamic_transform.translation - kinematic_transform.translation).length();
        assert!(
            divergence < bound,
            "low-speed divergence {divergence:.3} m exceeds the CG-offset bound {bound:.3} m"
        );
    }

    #[test]
    fn tire_slip_widens_the_line_as_speed_rises() {
        // Identical steering at rising speeds; the no-slip model would keep the turn
        // radius constant, tire slip must widen it. Gentle enough that neither axle
        // reaches the friction limit: the widening is pure slip, not saturation.
        let steering = 0.08;
        let radius_at = |speed: f64| {
            let mut world = World::new();
            let vehicle = spawn_dynamic_vehicle(
                &mut world,
                hot_lap_drive(speed, steering),
                VehicleDynamics::default(),
            );
            step_seconds(&mut world, 6.0);
            let dynamics = world.get::<VehicleDynamics>(vehicle).unwrap();
            // Steady-state turn radius follows from speed over yaw rate.
            (speed / dynamics.yaw_rate_rad_s, *dynamics)
        };

        let (slow_radius, slow_dynamics) = radius_at(5.0);
        let (fast_radius, fast_dynamics) = radius_at(12.0);

        assert!(slow_radius > 0.0 && fast_radius > 0.0);
        assert!(
            fast_radius > slow_radius * 1.05,
            "line must widen with speed: {slow_radius:.2} m -> {fast_radius:.2} m"
        );
        // The widening comes from real slip angles, not from saturation.
        assert!(fast_dynamics.front_slip_rad.abs() > slow_dynamics.front_slip_rad.abs());
        assert!(!fast_dynamics.front_saturated);
    }

    #[test]
    fn friction_limit_saturates_the_front_axle_and_understeers() {
        // A hard corner at speed exceeds mu Fz on the front axle.
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            hot_lap_drive(24.0, 0.5),
            VehicleDynamics::default(),
        );
        step_seconds(&mut world, 4.0);

        let dynamics = *world.get::<VehicleDynamics>(vehicle).unwrap();
        assert!(dynamics.front_saturated, "front axle must saturate");

        // Saturated fronts cannot deliver the kinematic yaw rate: understeer.
        let kinematic_yaw = 24.0 / VehicleDynamics::default().wheelbase_m() * 0.5_f64.tan();
        assert!(
            dynamics.yaw_rate_rad_s < kinematic_yaw * 0.5,
            "yaw rate {:.3} should be far below the no-slip {:.3}",
            dynamics.yaw_rate_rad_s,
            kinematic_yaw
        );
    }

    #[test]
    fn load_transfer_shifts_grip_between_axles() {
        let dynamics = VehicleDynamics::default();
        let total = dynamics.static_front_load_n() + dynamics.static_rear_load_n();
        assert!((total - dynamics.mass_kg * 9.81).abs() < 1e-9);
        // The default sedan is nose-heavy: more static load on the front axle.
        assert!(dynamics.static_front_load_n() > dynamics.static_rear_load_n());
    }

    #[test]
    fn vehicle_dynamics_is_deterministic() {
        let run = || {
            let mut world = World::new();
            let vehicle = spawn_dynamic_vehicle(
                &mut world,
                hot_lap_drive(18.0, 0.35),
                VehicleDynamics::default(),
            );
            step_seconds(&mut world, 5.0);
            (
                world.get::<Transform3>(vehicle).unwrap().translation,
                *world.get::<VehicleDynamics>(vehicle).unwrap(),
            )
        };

        assert_eq!(run(), run());
    }

    #[test]
    fn steering_lag_delays_the_response_and_zero_lag_matches_legacy() {
        let steering_after = |lag_s: f64, seconds: f64| {
            let mut world = World::new();
            let vehicle = spawn_dynamic_vehicle(
                &mut world,
                AckermannDrive {
                    target_steering_rad: 0.3,
                    speed_m_s: 10.0,
                    target_speed_m_s: 10.0,
                    max_speed_m_s: 30.0,
                    // High enough that the rate limit never binds: this test isolates
                    // the first-order lag. Their composition is covered implicitly by
                    // every other dynamic-model test using the default rate.
                    max_steering_rate_rad_s: 100.0,
                    ..AckermannDrive::default()
                },
                VehicleDynamics {
                    steering_lag_s: lag_s,
                    ..VehicleDynamics::default()
                },
            );
            step_seconds(&mut world, seconds);
            world.get::<AckermannDrive>(vehicle).unwrap().steering_rad
        };

        // Without lag the rate limit alone reaches the target quickly.
        let instant = steering_after(0.0, 0.5);
        assert!((instant - 0.3).abs() < 1e-9);
        // One time constant reaches ~63 percent of the step.
        let lagged = steering_after(0.2, 0.2);
        assert!((lagged - 0.3 * 0.632).abs() < 0.01, "got {lagged}");
        // The lag converges eventually.
        assert!((steering_after(0.2, 2.0) - 0.3).abs() < 1e-3);
    }

    #[test]
    fn rigid_body_velocity_includes_the_lateral_component() {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            hot_lap_drive(12.0, 0.08),
            VehicleDynamics::default(),
        );
        step_seconds(&mut world, 3.0);

        let dynamics = *world.get::<VehicleDynamics>(vehicle).unwrap();
        let transform = *world.get::<Transform3>(vehicle).unwrap();
        let body = world.get::<RigidBody>(vehicle).unwrap();

        // Velocity is not aligned with the nose: the slip is visible in the world state,
        // which is what a mounted IMU or wheel-speed sensor would observe. The velocity
        // uses the mid-step attitude, so the comparison allows the half-step of yaw.
        let forward = transform.rotation * Vec3::X;
        let along = body.linear_velocity_m_s.dot(forward);
        let across = (body.linear_velocity_m_s - forward * along).length();
        assert!(dynamics.lateral_velocity_m_s.abs() > 0.01);
        assert!((across - dynamics.lateral_velocity_m_s.abs()).abs() < 0.05);
    }
}
