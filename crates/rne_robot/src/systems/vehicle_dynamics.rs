use super::*;

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

/// Evaluates one axle's cornering stiffness, applying [`CorneringStiffnessLoadSensitivity`]
/// when present.
///
/// Absent `sensitivity` returns `reference_stiffness_n_rad` unchanged, which keeps
/// [`vehicle_dynamics`]'s constant-stiffness path bit-for-bit identical to before this
/// adjustment existed. A non-positive `static_axle_load_n` also falls back to the
/// unchanged value; that should not occur once [`VehicleDynamics::is_valid`] holds, since
/// a valid spec has strictly positive mass and axle distances.
///
/// When present, stiffness scales affinely with the axle's instantaneous load ratio
/// relative to its own static load, reusing [`CombinedSlipTireSpec`]'s load-ratio clamp
/// through [`capped_load_ratio`] and its `load_sensitivity_per_load_ratio` functional
/// form, with the slope sign flipped: cornering stiffness increases with load where tire
/// friction decreases with it. Because the affine map has a nonzero intercept at
/// `load_ratio == 1`, it is sub-linear in the load ratio (doubling the ratio does not
/// double the output), and it evaluates to exactly `reference_stiffness_n_rad` at the
/// axle's static load, preserving that parameter's declared meaning. Validation bounds
/// `load_sensitivity_per_load_ratio` to `[0.0, 1.0)` and `load_ratio` is never negative,
/// so the result stays strictly positive.
pub(crate) fn effective_cornering_stiffness(
    reference_stiffness_n_rad: f64,
    axle_load_n: f64,
    static_axle_load_n: f64,
    sensitivity: Option<CorneringStiffnessLoadSensitivity>,
) -> f64 {
    let Some(sensitivity) = sensitivity else {
        return reference_stiffness_n_rad;
    };
    if static_axle_load_n <= 0.0 {
        return reference_stiffness_n_rad;
    }
    let load_ratio = capped_load_ratio(
        axle_load_n,
        static_axle_load_n,
        sensitivity.maximum_load_ratio,
    );
    let stiffness_ratio =
        (1.0 + sensitivity.load_sensitivity_per_load_ratio * (load_ratio - 1.0)).max(0.0);
    reference_stiffness_n_rad * stiffness_ratio
}

/// Lateral force from one axle, optionally split into two equal-slip tires to
/// represent left/right load transfer.
///
/// With `lateral_transfer_n == 0` the two half-load tires reproduce the
/// single-tire axle exactly, including the friction clamp and the saturation
/// flag: halving and doubling are exact in binary floating point, so the
/// per-side stiffness sums to the axle stiffness and the per-side clamps sum to
/// the axle clamp whenever both sides are in the same regime.
pub(crate) fn axle_lateral_force_n(
    axle_stiffness_n_rad: f64,
    static_axle_load_n: f64,
    axle_load_n: f64,
    slip_rad: f64,
    friction_coefficient: f64,
    sensitivity: Option<CorneringStiffnessLoadSensitivity>,
    lateral_transfer_n: f64,
) -> (f64, bool) {
    let half_load_n = 0.5 * axle_load_n;
    let half_static_n = 0.5 * static_axle_load_n;
    let reference_stiffness_n_rad = 0.5 * axle_stiffness_n_rad;
    let transfer_n = lateral_transfer_n.max(0.0);
    let outer_load_n = (half_load_n + transfer_n).max(0.0);
    let inner_load_n = (half_load_n - transfer_n).max(0.0);

    let mut saturated = false;
    let mut side_forces_n = [0.0_f64; 2];
    for (index, side_load_n) in [outer_load_n, inner_load_n].into_iter().enumerate() {
        let stiffness_n_rad = effective_cornering_stiffness(
            reference_stiffness_n_rad,
            side_load_n,
            half_static_n,
            sensitivity,
        );
        let limit_n = friction_coefficient * side_load_n;
        let demanded_n = -stiffness_n_rad * slip_rad;
        if demanded_n.abs() > limit_n {
            saturated = true;
        }
        side_forces_n[index] = demanded_n.clamp(-limit_n, limit_n);
    }
    // Summing the two sides directly, rather than from a `+0.0` accumulator, keeps
    // the sign of a zero-slip force identical to the single-tire formula.
    (side_forces_n[0] + side_forces_n[1], saturated)
}

/// Explicit per-wheel lateral result of the optional four-wheel model.
pub(crate) struct FourWheelLateralForces {
    /// Total body-lateral force `sum Fy_i cos(delta_i)`, in newtons.
    lateral_force_n: f64,
    /// Yaw moment `sum x_i Fy_i cos(delta_i)`, in newton meters.
    yaw_moment_nm: f64,
    /// Per-wheel slip angles in FL, FR, RL, RR order, in radians.
    slip_rad: [f64; 4],
    /// Per-wheel friction saturation in FL, FR, RL, RR order.
    saturated: [bool; 4],
}

/// Computes the explicit per-wheel lateral forces of [`VehicleDynamics::four_wheel`].
///
/// Wheel order is front-left, front-right, rear-left, rear-right, with the left wheels
/// at lateral offset `+track/2`. Each front wheel gets an Ackermann steer angle blended
/// by [`FourWheelVehicleSpec::ackermann_fraction`]; the rear wheels are unsteered. Each
/// wheel's forward speed is `vx + r z`, so the outer wheel of a turn runs faster, and its
/// lateral speed is `vy + r x`, the same per-axle value the single-track model uses.
///
/// Lateral load transfer is directional: the outer side is loaded according to the sign
/// of the steady centripetal acceleration `vx r`, using the same per-axle transfer value
/// `m |vx r| h / track` as the single-track split. This mirrors the bicycle model's
/// lateral-only simplification: the longitudinal force component of a steered front tire
/// and any aligning moment are intentionally omitted.
pub(crate) fn four_wheel_lateral_forces(
    dynamics: &VehicleDynamics,
    spec: FourWheelVehicleSpec,
    vx: f64,
    ax: f64,
    vy: f64,
    r: f64,
    delta: f64,
) -> FourWheelLateralForces {
    let front_axle_m = dynamics.front_axle_m;
    let rear_axle_m = dynamics.rear_axle_m;
    let wheelbase_m = dynamics.wheelbase_m();
    let half_track_m = 0.5 * spec.track_width_m;

    // Axle loads with longitudinal transfer; clamped so neither axle lifts.
    let transfer_n = dynamics.mass_kg * ax * dynamics.center_of_mass_height_m / wheelbase_m;
    let front_load_n = (dynamics.static_front_load_n() - transfer_n).max(0.0);
    let rear_load_n = (dynamics.static_rear_load_n() + transfer_n).max(0.0);

    // Lateral roll transfer, split front/rear and added to the outer side.
    let centripetal_acceleration_m_s2 = vx * r;
    let total_transfer_n =
        dynamics.mass_kg * centripetal_acceleration_m_s2.abs() * dynamics.center_of_mass_height_m
            / spec.track_width_m;
    let front_transfer_n = spec.front_roll_stiffness_fraction * total_transfer_n;
    let rear_transfer_n = (1.0 - spec.front_roll_stiffness_fraction) * total_transfer_n;
    let transfer_sign = if centripetal_acceleration_m_s2 >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let front_half_n = 0.5 * front_load_n;
    let rear_half_n = 0.5 * rear_load_n;

    // Rear-axle path radius; infinite for straight-line steering gives zero Ackermann
    // difference, so exact and parallel steering coincide there.
    let radius_m = if delta.abs() > 0.0 {
        wheelbase_m / delta.tan()
    } else {
        f64::INFINITY
    };
    let ackermann = |z_m: f64| {
        let denominator_m = radius_m + z_m;
        let exact_rad = if denominator_m.is_infinite() {
            0.0
        } else if denominator_m != 0.0 {
            (wheelbase_m / denominator_m).atan()
        } else {
            delta
        };
        delta + spec.ackermann_fraction * (exact_rad - delta)
    };
    let front_left_steer_rad = ackermann(half_track_m);
    let front_right_steer_rad = ackermann(-half_track_m);

    let wheel = |x_m: f64,
                 z_m: f64,
                 load_n: f64,
                 static_half_n: f64,
                 axle_stiffness_n_rad: f64,
                 steer_rad: f64| {
        let wheel_lateral_velocity_m_s = vy + r * x_m;
        let wheel_forward_velocity_m_s = vx + r * z_m;
        let slip_rad = (wheel_lateral_velocity_m_s / wheel_forward_velocity_m_s).atan() - steer_rad;
        let stiffness_n_rad = effective_cornering_stiffness(
            0.5 * axle_stiffness_n_rad,
            load_n,
            static_half_n,
            dynamics.cornering_stiffness_load_sensitivity,
        );
        let limit_n = dynamics.friction_coefficient * load_n;
        let demanded_n = -stiffness_n_rad * slip_rad;
        let saturated = demanded_n.abs() > limit_n;
        (demanded_n.clamp(-limit_n, limit_n), slip_rad, saturated)
    };

    let front_static_half_n = 0.5 * dynamics.static_front_load_n();
    let rear_static_half_n = 0.5 * dynamics.static_rear_load_n();

    let (front_left_force_n, front_left_slip_rad, front_left_saturated) = wheel(
        front_axle_m,
        half_track_m,
        (front_half_n + transfer_sign * front_transfer_n).max(0.0),
        front_static_half_n,
        dynamics.front_cornering_stiffness_n_rad,
        front_left_steer_rad,
    );
    let (front_right_force_n, front_right_slip_rad, front_right_saturated) = wheel(
        front_axle_m,
        -half_track_m,
        (front_half_n - transfer_sign * front_transfer_n).max(0.0),
        front_static_half_n,
        dynamics.front_cornering_stiffness_n_rad,
        front_right_steer_rad,
    );
    let (rear_left_force_n, rear_left_slip_rad, rear_left_saturated) = wheel(
        -rear_axle_m,
        half_track_m,
        (rear_half_n + transfer_sign * rear_transfer_n).max(0.0),
        rear_static_half_n,
        dynamics.rear_cornering_stiffness_n_rad,
        0.0,
    );
    let (rear_right_force_n, rear_right_slip_rad, rear_right_saturated) = wheel(
        -rear_axle_m,
        -half_track_m,
        (rear_half_n - transfer_sign * rear_transfer_n).max(0.0),
        rear_static_half_n,
        dynamics.rear_cornering_stiffness_n_rad,
        0.0,
    );

    let front_left_body_n = front_left_force_n * front_left_steer_rad.cos();
    let front_right_body_n = front_right_force_n * front_right_steer_rad.cos();
    FourWheelLateralForces {
        lateral_force_n: front_left_body_n
            + front_right_body_n
            + rear_left_force_n
            + rear_right_force_n,
        yaw_moment_nm: front_axle_m * (front_left_body_n + front_right_body_n)
            - rear_axle_m * (rear_left_force_n + rear_right_force_n),
        slip_rad: [
            front_left_slip_rad,
            front_right_slip_rad,
            rear_left_slip_rad,
            rear_right_slip_rad,
        ],
        saturated: [
            front_left_saturated,
            front_right_saturated,
            rear_left_saturated,
            rear_right_saturated,
        ],
    }
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
/// `C` itself is constant unless [`VehicleDynamics::cornering_stiffness_load_sensitivity`]
/// is present, in which case `effective_cornering_stiffness` scales it with the same
/// per-axle `Fz`.
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
            dynamics.wheel_slip_rad = [0.0; 4];
            dynamics.wheel_saturated = [false; 4];
        } else {
            let vy = dynamics.lateral_velocity_m_s;
            let r = dynamics.yaw_rate_rad_s;

            if let Some(four_wheel) = dynamics.four_wheel {
                let forces = four_wheel_lateral_forces(&dynamics, four_wheel, vx, ax, vy, r, delta);
                dynamics.front_slip_rad = 0.5 * (forces.slip_rad[0] + forces.slip_rad[1]);
                dynamics.rear_slip_rad = 0.5 * (forces.slip_rad[2] + forces.slip_rad[3]);
                dynamics.front_saturated = forces.saturated[0] || forces.saturated[1];
                dynamics.rear_saturated = forces.saturated[2] || forces.saturated[3];
                dynamics.wheel_slip_rad = forces.slip_rad;
                dynamics.wheel_saturated = forces.saturated;

                let lateral_acceleration = forces.lateral_force_n / dynamics.mass_kg - vx * r;
                let yaw_acceleration = forces.yaw_moment_nm / dynamics.yaw_inertia_kg_m2;

                dynamics.lateral_velocity_m_s += lateral_acceleration * dt_s;
                dynamics.yaw_rate_rad_s += yaw_acceleration * dt_s;
            } else {
                let alpha_f = ((vy + dynamics.front_axle_m * r) / vx).atan() - delta;
                let alpha_r = ((vy - dynamics.rear_axle_m * r) / vx).atan();

                // Left/right load transfer is opt-in. Using the steady centripetal
                // acceleration from the current state rather than this step's forces keeps
                // the load/force relation explicit and loop-free, mirroring the
                // longitudinal path that uses the previous chassis acceleration.
                let (front_transfer_n, rear_transfer_n) = match dynamics.lateral_load_transfer {
                    Some(spec) => {
                        let centripetal_acceleration_m_s2 = vx * r;
                        let total_transfer_n = dynamics.mass_kg
                            * centripetal_acceleration_m_s2.abs()
                            * dynamics.center_of_mass_height_m
                            / spec.track_width_m;
                        (
                            spec.front_roll_stiffness_fraction * total_transfer_n,
                            (1.0 - spec.front_roll_stiffness_fraction) * total_transfer_n,
                        )
                    }
                    None => (0.0, 0.0),
                };

                let (front_force_n, front_saturated) = axle_lateral_force_n(
                    dynamics.front_cornering_stiffness_n_rad,
                    dynamics.static_front_load_n(),
                    front_load_n,
                    alpha_f,
                    dynamics.friction_coefficient,
                    dynamics.cornering_stiffness_load_sensitivity,
                    front_transfer_n,
                );
                let (rear_force_n, rear_saturated) = axle_lateral_force_n(
                    dynamics.rear_cornering_stiffness_n_rad,
                    dynamics.static_rear_load_n(),
                    rear_load_n,
                    alpha_r,
                    dynamics.friction_coefficient,
                    dynamics.cornering_stiffness_load_sensitivity,
                    rear_transfer_n,
                );

                dynamics.front_slip_rad = alpha_f;
                dynamics.rear_slip_rad = alpha_r;
                dynamics.front_saturated = front_saturated;
                dynamics.rear_saturated = rear_saturated;
                dynamics.wheel_slip_rad = [0.0; 4];
                dynamics.wheel_saturated = [false; 4];

                let lateral_acceleration =
                    (front_force_n * delta.cos() + rear_force_n) / dynamics.mass_kg - vx * r;
                let yaw_acceleration = (dynamics.front_axle_m * front_force_n * delta.cos()
                    - dynamics.rear_axle_m * rear_force_n)
                    / dynamics.yaw_inertia_kg_m2;

                dynamics.lateral_velocity_m_s += lateral_acceleration * dt_s;
                dynamics.yaw_rate_rad_s += yaw_acceleration * dt_s;
            }
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

pub(crate) fn move_towards(current: f64, target: f64, max_delta: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

pub(crate) fn clamp_length(value: Vec3, max_length: f64) -> Vec3 {
    let length = value.length();
    if length > max_length && length > 0.0 {
        value * (max_length / length)
    } else {
        value
    }
}

pub(crate) fn wrap_angle_rad(mut angle_rad: f64) -> f64 {
    while angle_rad > std::f64::consts::PI {
        angle_rad -= std::f64::consts::TAU;
    }
    while angle_rad < -std::f64::consts::PI {
        angle_rad += std::f64::consts::TAU;
    }
    angle_rad
}
