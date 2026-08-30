//! Robot control systems.

use crate::actuator::ControlMode;
use crate::commands::{ActuatorCommand, ActuatorCommandBuffer};
use crate::components::{
    AckermannDrive, Actuator, CombinedSlipTireSpec, CombinedSlipTireState,
    DcMotorCompletedTelemetry, DcMotorFailureMode, DcMotorSpec, DcMotorState, Joint, JointKind,
    MultirotorFlight, TransmissionSpec, VehicleDynamics, WheelAssemblySpec,
};
use crate::diff_drive::DifferentialDrive;
use crate::joint::{validate_joint_position, validate_joint_velocity, JointValidationError};
use bevy_ecs::prelude::{Entity, World};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, ContactPointSample, ExternalBodyWrench, JointActuation, JointMotor,
    RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use thiserror::Error;

/// Invalid configuration or input supplied to a mobility-plant evaluator.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum MobilityPlantEvaluationError {
    /// A model specification failed its physical validity checks.
    #[error("invalid mobility plant specification")]
    InvalidSpec,
    /// A command or completed-state input was non-finite or outside its physical domain.
    #[error("invalid mobility plant input")]
    InvalidInput,
    /// The fixed step was zero, negative, or non-finite.
    #[error("mobility plant timestep must be finite and positive")]
    InvalidTimeStep,
}

/// Completed DC motor electrical and shaft-torque evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcMotorEvaluation {
    /// State to retain for the next completed step.
    pub state: DcMotorState,
    /// Voltage actually applied after supply limits and failure behavior, in volts.
    pub terminal_voltage_v: f64,
    /// Back-EMF at the supplied rotor speed, in volts.
    pub back_emf_v: f64,
    /// Electromagnetic torque before shaft losses, in newton-meters.
    pub electromagnetic_torque_nm: f64,
    /// Viscous plus Coulomb torque opposing the shaft, in newton-meters.
    pub shaft_loss_torque_nm: f64,
    /// Net torque available at the motor shaft, in newton-meters.
    pub shaft_torque_nm: f64,
    /// Whether the requested terminal voltage exceeded the supply limit.
    pub voltage_saturated: bool,
    /// Whether the unconstrained armature current exceeded the current limit.
    pub current_saturated: bool,
}

impl DcMotorEvaluation {
    /// Converts this completed evaluation into sensor-source telemetry.
    ///
    /// Temperature is supplied separately because the v1 electrical evaluator
    /// deliberately has no thermal state.
    pub fn completed_telemetry(
        self,
        failure_mode: DcMotorFailureMode,
        winding_temperature_c: Option<f64>,
    ) -> DcMotorCompletedTelemetry {
        DcMotorCompletedTelemetry {
            terminal_voltage_v: self.terminal_voltage_v,
            current_a: self.state.current_a,
            back_emf_v: self.back_emf_v,
            winding_temperature_c,
            voltage_saturated: self.voltage_saturated,
            current_saturated: self.current_saturated,
            failure_mode,
        }
    }
}

/// Completed static transmission evaluation at one wheel coordinate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransmissionEvaluation {
    /// Motor-shaft velocity implied by the wheel coordinate, in radians per second.
    pub motor_velocity_rad_s: f64,
    /// Wheel-side torque after ratio and directional efficiency, in newton-meters.
    pub wheel_torque_nm: f64,
    /// Motor rotor inertia reflected to the wheel coordinate, in kilogram square meters.
    pub reflected_rotor_inertia_kg_m2: f64,
    /// Efficiency selected from the direction of mechanical power flow.
    pub applied_efficiency_ratio: f64,
}

/// Load-weighted contact patch reconstructed from completed backend contact evidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelContactPatch {
    /// Wheel entity represented by this patch.
    pub wheel_entity: Entity,
    /// Load-weighted application point in world coordinates, in meters.
    pub point_world_m: Vec3,
    /// Load-weighted unit normal pointing from the road toward the wheel.
    pub normal_road_to_wheel_world: Vec3,
    /// Wheel-surface velocity relative to the road at the patch, in meters per second.
    pub wheel_relative_to_road_world_m_s: Vec3,
    /// Total step-average normal load carried by the patch, in newtons.
    pub normal_load_n: f64,
}

/// Completed force and state from one transient combined-slip tire step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CombinedSlipTireEvaluation {
    /// State to retain for the next completed tire step.
    pub state: CombinedSlipTireState,
    /// Longitudinal force on the wheel in its positive-forward direction, in newtons.
    pub longitudinal_force_n: f64,
    /// Lateral force on the wheel in its positive-lateral direction, in newtons.
    pub lateral_force_n: f64,
    /// Load-sensitive longitudinal force limit after road scaling, in newtons.
    pub longitudinal_peak_force_n: f64,
    /// Load-sensitive lateral force limit after road scaling, in newtons.
    pub lateral_peak_force_n: f64,
    /// Combined utilization of the friction ellipse, bounded by one.
    pub friction_utilization: f64,
}

/// Completed contact and wheel-frame inputs for one combined-slip tire step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CombinedSlipTireInput {
    /// Aggregated completed-step contact, or `None` when the wheel is lifted.
    pub patch: Option<WheelContactPatch>,
    /// Positive wheel-forward unit axis in world coordinates.
    pub forward_world: Vec3,
    /// Positive wheel-lateral unit axis in world coordinates.
    pub lateral_world: Vec3,
    /// Signed wheel circumference speed from its completed angular coordinate, in meters per second.
    pub wheel_circumferential_speed_m_s: f64,
    /// Non-negative road friction multiplier for this patch.
    pub road_friction_scale: f64,
}

/// Aggregates deterministic point-contact evidence for one wheel.
///
/// `forward_world` and `lateral_world` must be finite, unit length, and orthogonal.
/// Samples not containing `wheel_entity`, non-positive loads, and non-finite samples are
/// ignored. The returned surface velocity and normal always use the road-to-wheel
/// convention, independently of canonical entity ordering.
pub fn aggregate_wheel_contact_patch(
    wheel_entity: Entity,
    samples: &[ContactPointSample],
    forward_world: Vec3,
    lateral_world: Vec3,
) -> Result<Option<WheelContactPatch>, MobilityPlantEvaluationError> {
    const AXIS_TOLERANCE: f64 = 1.0e-6;
    if !forward_world.is_finite()
        || !lateral_world.is_finite()
        || (forward_world.length() - 1.0).abs() > AXIS_TOLERANCE
        || (lateral_world.length() - 1.0).abs() > AXIS_TOLERANCE
        || forward_world.dot(lateral_world).abs() > AXIS_TOLERANCE
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }

    let mut normal_load_n = 0.0;
    let mut weighted_point = Vec3::ZERO;
    let mut weighted_normal = Vec3::ZERO;
    let mut weighted_velocity = Vec3::ZERO;
    for sample in samples {
        if sample.normal_force_n <= 0.0
            || !sample.normal_force_n.is_finite()
            || !sample.point_world_m.is_finite()
            || !sample.normal_a_to_b.is_finite()
            || !sample.velocity_b_relative_to_a_world_m_s.is_finite()
        {
            continue;
        }
        let (normal_road_to_wheel, wheel_relative_to_road) = if sample.entity_a == wheel_entity {
            (
                -sample.normal_a_to_b,
                -sample.velocity_b_relative_to_a_world_m_s,
            )
        } else if sample.entity_b == wheel_entity {
            (
                sample.normal_a_to_b,
                sample.velocity_b_relative_to_a_world_m_s,
            )
        } else {
            continue;
        };
        let weight = sample.normal_force_n;
        normal_load_n += weight;
        weighted_point += sample.point_world_m * weight;
        weighted_normal += normal_road_to_wheel * weight;
        weighted_velocity += wheel_relative_to_road * weight;
    }
    if normal_load_n == 0.0 {
        return Ok(None);
    }
    let normal = weighted_normal.normalize_or_zero();
    if normal == Vec3::ZERO {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(Some(WheelContactPatch {
        wheel_entity,
        point_world_m: weighted_point / normal_load_n,
        normal_road_to_wheel_world: normal,
        wheel_relative_to_road_world_m_s: weighted_velocity / normal_load_n,
        normal_load_n,
    }))
}

/// Evaluates one deterministic, identifiable transient combined-slip tire force.
///
/// The contact velocity already includes wheel rotation. Positive longitudinal slip is
/// therefore `-surface_velocity / (abs(circumferential_speed) + v_num)`, matching the
/// low-speed-safe convention used by handling-oriented tire models. `road_friction_scale`
/// enables spatial or randomized road friction without changing the identified tire spec.
pub fn evaluate_combined_slip_tire(
    spec: CombinedSlipTireSpec,
    state: CombinedSlipTireState,
    input: CombinedSlipTireInput,
    dt_s: f64,
) -> Result<CombinedSlipTireEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.longitudinal_slip_ratio.is_finite()
        || !state.lateral_slip_tangent.is_finite()
        || !input.wheel_circumferential_speed_m_s.is_finite()
        || !input.road_friction_scale.is_finite()
        || input.road_friction_scale < 0.0
        || !input.forward_world.is_finite()
        || !input.lateral_world.is_finite()
        || (input.forward_world.length() - 1.0).abs() > 1.0e-6
        || (input.lateral_world.length() - 1.0).abs() > 1.0e-6
        || input.forward_world.dot(input.lateral_world).abs() > 1.0e-6
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }
    let Some(patch) = input.patch else {
        return Ok(zero_tire_evaluation());
    };
    if !patch.point_world_m.is_finite()
        || !patch.normal_road_to_wheel_world.is_finite()
        || !patch.wheel_relative_to_road_world_m_s.is_finite()
        || !patch.normal_load_n.is_finite()
        || patch.normal_load_n <= 0.0
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }

    let transport_speed_m_s =
        input.wheel_circumferential_speed_m_s.abs() + spec.low_speed_regularization_m_s;
    let longitudinal_surface_speed_m_s = patch
        .wheel_relative_to_road_world_m_s
        .dot(input.forward_world);
    let lateral_surface_speed_m_s = patch
        .wheel_relative_to_road_world_m_s
        .dot(input.lateral_world);
    let target_longitudinal_slip_ratio = -longitudinal_surface_speed_m_s / transport_speed_m_s;
    let target_lateral_slip_tangent = -lateral_surface_speed_m_s / transport_speed_m_s;
    let next_longitudinal_slip = relax_slip(
        state.longitudinal_slip_ratio,
        target_longitudinal_slip_ratio,
        spec.longitudinal_relaxation_length_m,
        transport_speed_m_s,
        dt_s,
    );
    let next_lateral_slip = relax_slip(
        state.lateral_slip_tangent,
        target_lateral_slip_tangent,
        spec.lateral_relaxation_length_m,
        transport_speed_m_s,
        dt_s,
    );

    let uncapped_load_ratio = patch.normal_load_n / spec.reference_load_n;
    let load_ratio = uncapped_load_ratio.min(spec.maximum_load_ratio);
    let friction_ratio = (1.0 - spec.load_sensitivity_per_load_ratio * (load_ratio - 1.0))
        .max(spec.minimum_friction_ratio);
    let longitudinal_peak_force_n = spec.longitudinal_peak_friction
        * friction_ratio
        * patch.normal_load_n
        * input.road_friction_scale;
    let lateral_peak_force_n = spec.lateral_peak_friction
        * friction_ratio
        * patch.normal_load_n
        * input.road_friction_scale;
    let raw_longitudinal_force_n =
        spec.longitudinal_stiffness_n * load_ratio * next_longitudinal_slip;
    let raw_lateral_force_n = spec.lateral_stiffness_n * load_ratio * next_lateral_slip;
    let normalized_longitudinal = if longitudinal_peak_force_n > 0.0 {
        raw_longitudinal_force_n / longitudinal_peak_force_n
    } else {
        0.0
    };
    let normalized_lateral = if lateral_peak_force_n > 0.0 {
        raw_lateral_force_n / lateral_peak_force_n
    } else {
        0.0
    };
    let demand = normalized_longitudinal.hypot(normalized_lateral);
    let force_scale = if demand > 1.0e-12 {
        demand.tanh() / demand
    } else {
        1.0
    };
    let force_scale = if input.road_friction_scale == 0.0 {
        0.0
    } else {
        force_scale
    };
    Ok(CombinedSlipTireEvaluation {
        state: CombinedSlipTireState {
            longitudinal_slip_ratio: next_longitudinal_slip,
            lateral_slip_tangent: next_lateral_slip,
        },
        longitudinal_force_n: raw_longitudinal_force_n * force_scale,
        lateral_force_n: raw_lateral_force_n * force_scale,
        longitudinal_peak_force_n,
        lateral_peak_force_n,
        friction_utilization: demand.tanh(),
    })
}

/// Converts a tire evaluation into the backend-neutral one-step wrench boundary.
pub fn combined_slip_tire_wrench(
    patch: WheelContactPatch,
    evaluation: CombinedSlipTireEvaluation,
    forward_world: Vec3,
    lateral_world: Vec3,
) -> Result<ExternalBodyWrench, MobilityPlantEvaluationError> {
    if !forward_world.is_finite()
        || !lateral_world.is_finite()
        || !evaluation.longitudinal_force_n.is_finite()
        || !evaluation.lateral_force_n.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let wrench = ExternalBodyWrench {
        entity: patch.wheel_entity,
        point_world_m: patch.point_world_m,
        force_world_n: forward_world * evaluation.longitudinal_force_n
            + lateral_world * evaluation.lateral_force_n,
        torque_world_nm: Vec3::ZERO,
    };
    if !wrench.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(wrench)
}

fn relax_slip(current: f64, target: f64, length_m: f64, speed_m_s: f64, dt_s: f64) -> f64 {
    if length_m == 0.0 {
        target
    } else {
        let fraction = 1.0 - (-speed_m_s * dt_s / length_m).exp();
        current + fraction * (target - current)
    }
}

fn zero_tire_evaluation() -> CombinedSlipTireEvaluation {
    CombinedSlipTireEvaluation {
        state: CombinedSlipTireState::default(),
        longitudinal_force_n: 0.0,
        lateral_force_n: 0.0,
        longitudinal_peak_force_n: 0.0,
        lateral_peak_force_n: 0.0,
        friction_utilization: 0.0,
    }
}

/// Evaluates one DC motor from terminal voltage and completed rotor velocity.
///
/// With no inductance, current is the algebraic equivalent-circuit solution
/// `I = (V - k_e omega) / R`. With inductance, current advances by explicit Euler from
/// `dI/dt = (V - k_e omega - R I) / L` and is then limited. Shaft loss combines viscous
/// friction and a regularized Coulomb term: at standstill Coulomb friction cancels available
/// electromagnetic torque up to its declared magnitude rather than inventing motion.
pub fn evaluate_dc_motor(
    spec: DcMotorSpec,
    state: DcMotorState,
    command_voltage_v: f64,
    rotor_velocity_rad_s: f64,
    dt_s: f64,
) -> Result<DcMotorEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.current_a.is_finite()
        || !command_voltage_v.is_finite()
        || !rotor_velocity_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }

    let voltage_saturated = command_voltage_v.abs() > spec.supply_voltage_v;
    let limited_command_voltage_v =
        command_voltage_v.clamp(-spec.supply_voltage_v, spec.supply_voltage_v);
    let terminal_voltage_v = match spec.failure_mode {
        DcMotorFailureMode::Nominal => limited_command_voltage_v,
        DcMotorFailureMode::OpenCircuit | DcMotorFailureMode::ShortCircuit => 0.0,
    };
    let back_emf_v = spec.back_emf_constant_v_s_rad * rotor_velocity_rad_s;
    let unconstrained_current_a = match (spec.failure_mode, spec.inductance_h) {
        (DcMotorFailureMode::OpenCircuit, _) => 0.0,
        (_, Some(inductance_h)) => {
            state.current_a
                + (terminal_voltage_v - back_emf_v - spec.resistance_ohm * state.current_a)
                    / inductance_h
                    * dt_s
        }
        (_, None) => (terminal_voltage_v - back_emf_v) / spec.resistance_ohm,
    };
    let current_saturated = unconstrained_current_a.abs() > spec.current_limit_a;
    let current_a = unconstrained_current_a.clamp(-spec.current_limit_a, spec.current_limit_a);
    let electromagnetic_torque_nm = spec.torque_constant_nm_a * current_a;
    let viscous_loss_nm = spec.viscous_friction_nm_s_rad * rotor_velocity_rad_s;
    let coulomb_loss_nm = if rotor_velocity_rad_s.abs() > 1.0e-12 {
        spec.coulomb_friction_nm * rotor_velocity_rad_s.signum()
    } else {
        electromagnetic_torque_nm.clamp(-spec.coulomb_friction_nm, spec.coulomb_friction_nm)
    };
    let shaft_loss_torque_nm = viscous_loss_nm + coulomb_loss_nm;

    Ok(DcMotorEvaluation {
        state: DcMotorState { current_a },
        terminal_voltage_v,
        back_emf_v,
        electromagnetic_torque_nm,
        shaft_loss_torque_nm,
        shaft_torque_nm: electromagnetic_torque_nm - shaft_loss_torque_nm,
        voltage_saturated,
        current_saturated,
    })
}

/// Maps motor torque and rotor inertia to a wheel coordinate without backend types.
///
/// This is the rigid static map. Declared backlash and compliance require a later stateful
/// driveline evaluator and are intentionally not approximated by hidden backend joints.
pub fn evaluate_transmission(
    spec: TransmissionSpec,
    motor_rotor_inertia_kg_m2: f64,
    motor_torque_nm: f64,
    wheel_velocity_rad_s: f64,
) -> Result<TransmissionEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !motor_rotor_inertia_kg_m2.is_finite()
        || motor_rotor_inertia_kg_m2 < 0.0
        || !motor_torque_nm.is_finite()
        || !wheel_velocity_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let ratio = spec.ratio_motor_rad_per_wheel_rad;
    let motor_velocity_rad_s = wheel_velocity_rad_s * ratio;
    let applied_efficiency_ratio = if motor_torque_nm * motor_velocity_rad_s >= 0.0 {
        spec.drive_efficiency_ratio
    } else {
        spec.backdrive_efficiency_ratio
    };
    Ok(TransmissionEvaluation {
        motor_velocity_rad_s,
        wheel_torque_nm: motor_torque_nm * ratio * applied_efficiency_ratio,
        reflected_rotor_inertia_kg_m2: motor_rotor_inertia_kg_m2 * ratio * ratio,
        applied_efficiency_ratio,
    })
}

/// Returns wheel rolling-resistance torque opposing completed wheel motion.
///
/// The v1 law is `Crr * normal_load * radius` and returns zero at exact standstill so it
/// cannot create a direction. A later wheel/ground solver may use impending slip to model
/// static rolling resistance.
pub fn wheel_rolling_resistance_torque_nm(
    spec: WheelAssemblySpec,
    normal_load_n: f64,
    wheel_velocity_rad_s: f64,
) -> Result<f64, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !normal_load_n.is_finite() || normal_load_n < 0.0 || !wheel_velocity_rad_s.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(if wheel_velocity_rad_s == 0.0 {
        0.0
    } else {
        -wheel_velocity_rad_s.signum()
            * spec.rolling_resistance_coefficient
            * normal_load_n
            * spec.radius_m
    })
}

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
            integrate_kinematic_wheel_joints(world, drive, dt_s);
            sync_wheel_transforms(world, drive, &base_snapshot);
        }

        if let Some(mut body) = world.get_mut::<RigidBody>(drive.base_link) {
            let forward_flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
            body.linear_velocity_m_s = forward_flat * linear_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rad_s, 0.0);
        }
    }
}

fn integrate_kinematic_wheel_joints(world: &mut World, drive: &DifferentialDrive, dt_s: f64) {
    for actuator_entity in [drive.left_actuator, drive.right_actuator] {
        let Some(joint_entity) = world
            .get::<Actuator>(actuator_entity)
            .and_then(|actuator| actuator.joint)
        else {
            continue;
        };
        if let Some(mut joint) = world.get_mut::<Joint>(joint_entity) {
            joint.position += joint.velocity * dt_s;
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

/// Copies every actuator target into unit-explicit [`JointActuation`].
///
/// The optional `drives` argument on [`sync_joint_motors_from_actuators`] is kept
/// for source compatibility with older diff-drive callers. Named URDF actuators
/// use this function directly and are resolved through their [`Joint`] child link.
/// Existing [`JointMotor`] components are updated as a compatibility path.
pub fn sync_all_joint_motors_from_actuators(world: &mut World) {
    let mut actuator_entities: Vec<_> = world
        .iter_entities()
        .map(|entity| entity.id())
        .filter(|entity| world.get::<Actuator>(*entity).is_some())
        .collect();
    actuator_entities.sort_unstable();

    for actuator_entity in actuator_entities {
        let Some((joint_entity, mode, target, limits)) =
            world.get::<Actuator>(actuator_entity).map(|actuator| {
                (
                    actuator.joint,
                    actuator.mode,
                    actuator.target,
                    actuator.limits,
                )
            })
        else {
            continue;
        };
        let Some(joint_entity) = joint_entity else {
            continue;
        };
        let Some((child_link, joint_kind)) = world
            .get::<Joint>(joint_entity)
            .map(|joint| (joint.child_link, joint.kind))
        else {
            continue;
        };
        let tuning = world
            .get::<JointMotor>(child_link)
            .copied()
            .unwrap_or_default();
        let max_output = if limits.max_effort_nm.is_finite() {
            limits.max_effort_nm.max(0.0)
        } else {
            0.0
        };
        let stiffness = if tuning.stiffness.is_finite() && tuning.stiffness > 0.0 {
            tuning.stiffness
        } else {
            40.0
        };
        let gain = if tuning.gain.is_finite() && tuning.gain >= 0.0 {
            tuning.gain
        } else {
            1.0
        };
        let actuation = match (joint_kind, mode) {
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Position) => {
                JointActuation::RevolutePosition {
                    target_position_rad: target.position_rad,
                    stiffness_nm_per_rad: stiffness,
                    damping_nm_s_per_rad: gain,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Velocity) => {
                JointActuation::RevoluteVelocity {
                    target_velocity_rad_s: target.velocity_rad_s,
                    gain_nm_s_per_rad: gain,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Effort) => {
                JointActuation::RevoluteEffort {
                    effort_nm: target.effort_nm,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Prismatic, ControlMode::Position) => JointActuation::PrismaticPosition {
                target_position_m: target.position_rad,
                stiffness_n_per_m: stiffness,
                damping_n_s_per_m: gain,
                max_force_n: max_output,
            },
            (JointKind::Prismatic, ControlMode::Velocity) => JointActuation::PrismaticVelocity {
                target_velocity_m_s: target.velocity_rad_s,
                gain_n_s_per_m: gain,
                max_force_n: max_output,
            },
            (JointKind::Prismatic, ControlMode::Effort) => JointActuation::PrismaticEffort {
                force_n: target.effort_nm,
                max_force_n: max_output,
            },
            (JointKind::Fixed, _) => JointActuation::Disabled,
        };
        world.entity_mut(child_link).insert(actuation);
        if let Some(mut motor) = world.get_mut::<JointMotor>(child_link) {
            motor.velocity_rad_s = match mode {
                ControlMode::Velocity => target.velocity_rad_s,
                ControlMode::Position | ControlMode::Effort => 0.0,
            };
            if mode == ControlMode::Position {
                motor.target_position = target.position_rad;
                motor.stiffness = stiffness;
            }
        }
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
    fn dc_motor_locked_rotor_obeys_current_and_voltage_limits() {
        let evaluation = evaluate_dc_motor(
            DcMotorSpec::default(),
            DcMotorState::default(),
            48.0,
            0.0,
            0.001,
        )
        .unwrap();

        assert_eq!(evaluation.terminal_voltage_v, 24.0);
        assert_eq!(evaluation.state.current_a, 20.0);
        let telemetry = evaluation.completed_telemetry(DcMotorFailureMode::Nominal, None);
        assert_eq!(telemetry.terminal_voltage_v, 24.0);
        assert_eq!(telemetry.current_a, 20.0);
        assert_eq!(telemetry.winding_temperature_c, None);
        assert!(telemetry.current_saturated);
        assert_eq!(evaluation.electromagnetic_torque_nm, 1.6);
        assert_eq!(evaluation.shaft_loss_torque_nm, 0.01);
        assert_eq!(evaluation.shaft_torque_nm, 1.59);
        assert!(evaluation.voltage_saturated);
        assert!(evaluation.current_saturated);
    }

    #[test]
    fn dc_motor_back_emf_and_failures_are_explicit() {
        let spec = DcMotorSpec::default();
        let free_speed_rad_s = spec.supply_voltage_v / spec.back_emf_constant_v_s_rad;
        let nominal = evaluate_dc_motor(
            spec,
            DcMotorState::default(),
            spec.supply_voltage_v,
            free_speed_rad_s,
            0.001,
        )
        .unwrap();
        assert_eq!(nominal.state.current_a, 0.0);
        assert!(nominal.shaft_torque_nm < 0.0);

        let open = evaluate_dc_motor(
            DcMotorSpec {
                failure_mode: DcMotorFailureMode::OpenCircuit,
                ..spec
            },
            DcMotorState { current_a: 5.0 },
            24.0,
            100.0,
            0.001,
        )
        .unwrap();
        assert_eq!(open.state.current_a, 0.0);
        assert_eq!(open.electromagnetic_torque_nm, 0.0);

        let short = evaluate_dc_motor(
            DcMotorSpec {
                failure_mode: DcMotorFailureMode::ShortCircuit,
                ..spec
            },
            DcMotorState::default(),
            24.0,
            100.0,
            0.001,
        )
        .unwrap();
        assert!(short.state.current_a < 0.0);
        assert!(short.shaft_torque_nm < 0.0);
    }

    #[test]
    fn dc_motor_inductance_retains_deterministic_current_state() {
        let spec = DcMotorSpec {
            resistance_ohm: 1.0,
            torque_constant_nm_a: 1.0,
            back_emf_constant_v_s_rad: 1.0,
            supply_voltage_v: 12.0,
            current_limit_a: 20.0,
            viscous_friction_nm_s_rad: 0.0,
            coulomb_friction_nm: 0.0,
            inductance_h: Some(0.1),
            ..DcMotorSpec::default()
        };
        let first = evaluate_dc_motor(spec, DcMotorState::default(), 1.0, 0.0, 0.01).unwrap();
        let second = evaluate_dc_motor(spec, first.state, 1.0, 0.0, 0.01).unwrap();
        assert!((first.state.current_a - 0.1).abs() < 1.0e-12);
        assert!((second.state.current_a - 0.19).abs() < 1.0e-12);
    }

    #[test]
    fn transmission_maps_directional_efficiency_and_reflected_inertia() {
        let spec = TransmissionSpec::default();
        let drive = evaluate_transmission(spec, 0.001, 1.0, 2.0).unwrap();
        assert_eq!(drive.motor_velocity_rad_s, 40.0);
        assert_eq!(drive.wheel_torque_nm, 18.0);
        assert_eq!(drive.reflected_rotor_inertia_kg_m2, 0.4);
        assert_eq!(drive.applied_efficiency_ratio, 0.9);

        let backdrive = evaluate_transmission(spec, 0.001, -1.0, 2.0).unwrap();
        assert_eq!(backdrive.wheel_torque_nm, -15.0);
        assert_eq!(backdrive.applied_efficiency_ratio, 0.75);
    }

    #[test]
    fn wheel_rolling_resistance_opposes_motion_without_inventing_direction() {
        let spec = WheelAssemblySpec::default();
        assert_eq!(
            wheel_rolling_resistance_torque_nm(spec, 100.0, 0.0).unwrap(),
            0.0
        );
        let forward = wheel_rolling_resistance_torque_nm(spec, 100.0, 2.0).unwrap();
        let reverse = wheel_rolling_resistance_torque_nm(spec, 100.0, -2.0).unwrap();
        assert!((forward + 0.15).abs() < 1.0e-12);
        assert!((reverse - 0.15).abs() < 1.0e-12);
    }

    #[test]
    fn mobility_plant_evaluators_reject_invalid_specs_and_inputs() {
        let invalid_motor = DcMotorSpec {
            resistance_ohm: 0.0,
            ..DcMotorSpec::default()
        };
        assert_eq!(
            evaluate_dc_motor(invalid_motor, DcMotorState::default(), 0.0, 0.0, 0.001),
            Err(MobilityPlantEvaluationError::InvalidSpec)
        );
        assert_eq!(
            evaluate_transmission(TransmissionSpec::default(), 0.001, f64::NAN, 0.0),
            Err(MobilityPlantEvaluationError::InvalidInput)
        );
        assert_eq!(
            wheel_rolling_resistance_torque_nm(WheelAssemblySpec::default(), -1.0, 0.0),
            Err(MobilityPlantEvaluationError::InvalidInput)
        );
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
    fn actuator_modes_map_to_unit_explicit_physics_commands() {
        let (mut world, _, joint, actuator) = setup_robot_with_joint();
        {
            let mut actuator = world.get_mut::<Actuator>(actuator).unwrap();
            actuator.mode = ControlMode::Position;
            actuator.target.position_rad = 0.4;
        }
        sync_all_joint_motors_from_actuators(&mut world);
        assert!(matches!(
            world.get::<JointActuation>(joint),
            Some(JointActuation::RevolutePosition {
                target_position_rad: 0.4,
                ..
            })
        ));

        {
            let mut actuator = world.get_mut::<Actuator>(actuator).unwrap();
            actuator.mode = ControlMode::Effort;
            actuator.target.effort_nm = 12.0;
        }
        sync_all_joint_motors_from_actuators(&mut world);
        assert_eq!(
            world.get::<JointActuation>(joint),
            Some(&JointActuation::RevoluteEffort {
                effort_nm: 12.0,
                max_effort_nm: 100.0,
            })
        );
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
        for wheel in [spawned.left_wheel, spawned.right_wheel] {
            let joint = world.get::<Joint>(wheel).unwrap();
            assert_eq!(joint.position, 5.0);
            assert_eq!(joint.velocity, 5.0);
        }
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

    fn test_patch(wheel_entity: Entity, velocity_m_s: Vec3, load_n: f64) -> WheelContactPatch {
        WheelContactPatch {
            wheel_entity,
            point_world_m: Vec3::new(0.0, 0.0, 0.0),
            normal_road_to_wheel_world: Vec3::Y,
            wheel_relative_to_road_world_m_s: velocity_m_s,
            normal_load_n: load_n,
        }
    }

    fn test_tire_input(
        patch: Option<WheelContactPatch>,
        wheel_circumferential_speed_m_s: f64,
        road_friction_scale: f64,
    ) -> CombinedSlipTireInput {
        CombinedSlipTireInput {
            patch,
            forward_world: Vec3::X,
            lateral_world: Vec3::Z,
            wheel_circumferential_speed_m_s,
            road_friction_scale,
        }
    }

    #[test]
    fn contact_patch_normalizes_canonical_entity_orientation() {
        let mut world = World::new();
        let road = world.spawn_empty().id();
        let wheel = world.spawn_empty().id();
        let samples = [
            ContactPointSample {
                entity_a: road,
                entity_b: wheel,
                point_world_m: Vec3::new(-0.1, 0.0, 0.0),
                normal_a_to_b: Vec3::Y,
                velocity_b_relative_to_a_world_m_s: Vec3::new(-2.0, 0.0, 0.5),
                normal_force_n: 300.0,
            },
            ContactPointSample {
                entity_a: road,
                entity_b: wheel,
                point_world_m: Vec3::new(0.1, 0.0, 0.0),
                normal_a_to_b: Vec3::Y,
                velocity_b_relative_to_a_world_m_s: Vec3::new(-1.0, 0.0, 0.5),
                normal_force_n: 100.0,
            },
        ];
        let patch = aggregate_wheel_contact_patch(wheel, &samples, Vec3::X, Vec3::Z)
            .unwrap()
            .unwrap();
        assert_eq!(patch.normal_load_n, 400.0);
        assert_eq!(patch.point_world_m, Vec3::new(-0.05, 0.0, 0.0));
        assert_eq!(patch.normal_road_to_wheel_world, Vec3::Y);
        assert_eq!(
            patch.wheel_relative_to_road_world_m_s,
            Vec3::new(-1.75, 0.0, 0.5)
        );

        let inverted = [ContactPointSample {
            entity_a: wheel,
            entity_b: road,
            point_world_m: Vec3::ZERO,
            normal_a_to_b: Vec3::NEG_Y,
            velocity_b_relative_to_a_world_m_s: Vec3::new(2.0, 0.0, -0.5),
            normal_force_n: 400.0,
        }];
        let inverted_patch = aggregate_wheel_contact_patch(wheel, &inverted, Vec3::X, Vec3::Z)
            .unwrap()
            .unwrap();
        assert_eq!(inverted_patch.normal_road_to_wheel_world, Vec3::Y);
        assert_eq!(
            inverted_patch.wheel_relative_to_road_world_m_s,
            Vec3::new(-2.0, 0.0, 0.5)
        );
    }

    #[test]
    fn combined_slip_force_has_physical_sign_and_bounded_ellipse() {
        let mut world = World::new();
        let wheel = world.spawn_empty().id();
        let spec = CombinedSlipTireSpec {
            longitudinal_relaxation_length_m: 0.0,
            lateral_relaxation_length_m: 0.0,
            ..CombinedSlipTireSpec::default()
        };
        let evaluation = evaluate_combined_slip_tire(
            spec,
            CombinedSlipTireState::default(),
            test_tire_input(
                Some(test_patch(wheel, Vec3::new(-5.0, 0.0, 3.0), 1_000.0)),
                10.0,
                1.0,
            ),
            0.01,
        )
        .unwrap();
        assert!(evaluation.longitudinal_force_n > 0.0);
        assert!(evaluation.lateral_force_n < 0.0);
        assert!(evaluation.friction_utilization <= 1.0);
        let ellipse = (evaluation.longitudinal_force_n / evaluation.longitudinal_peak_force_n)
            .hypot(evaluation.lateral_force_n / evaluation.lateral_peak_force_n);
        assert!(ellipse <= 1.0);

        let repeat = evaluate_combined_slip_tire(
            spec,
            CombinedSlipTireState::default(),
            test_tire_input(
                Some(test_patch(wheel, Vec3::new(-5.0, 0.0, 3.0), 1_000.0)),
                10.0,
                1.0,
            ),
            0.01,
        )
        .unwrap();
        assert_eq!(evaluation, repeat);
    }

    #[test]
    fn road_scale_and_load_sensitivity_change_available_force() {
        let mut world = World::new();
        let wheel = world.spawn_empty().id();
        let spec = CombinedSlipTireSpec {
            longitudinal_relaxation_length_m: 0.0,
            lateral_relaxation_length_m: 0.0,
            ..CombinedSlipTireSpec::default()
        };
        let evaluate = |load_n, road_scale| {
            evaluate_combined_slip_tire(
                spec,
                CombinedSlipTireState::default(),
                test_tire_input(
                    Some(test_patch(wheel, Vec3::new(-20.0, 0.0, 0.0), load_n)),
                    10.0,
                    road_scale,
                ),
                0.01,
            )
            .unwrap()
        };
        let dry = evaluate(1_000.0, 1.0);
        let split_low = evaluate(1_000.0, 0.4);
        assert!(split_low.longitudinal_force_n < dry.longitudinal_force_n);
        assert_eq!(
            split_low.longitudinal_peak_force_n,
            dry.longitudinal_peak_force_n * 0.4
        );
        let double_load = evaluate(2_000.0, 1.0);
        assert!(double_load.longitudinal_peak_force_n < dry.longitudinal_peak_force_n * 2.0);
    }

    #[test]
    fn low_speed_relaxation_and_lift_off_are_explicit() {
        let mut world = World::new();
        let wheel = world.spawn_empty().id();
        let spec = CombinedSlipTireSpec::default();
        let first = evaluate_combined_slip_tire(
            spec,
            CombinedSlipTireState::default(),
            test_tire_input(
                Some(test_patch(wheel, Vec3::new(-0.01, 0.0, 0.0), 800.0)),
                0.0,
                1.0,
            ),
            0.01,
        )
        .unwrap();
        assert!(first.state.longitudinal_slip_ratio.is_finite());
        assert!(first.state.longitudinal_slip_ratio > 0.0);
        assert!(first.state.longitudinal_slip_ratio < 0.1);

        let second = evaluate_combined_slip_tire(
            spec,
            first.state,
            test_tire_input(
                Some(test_patch(wheel, Vec3::new(-0.01, 0.0, 0.0), 800.0)),
                0.0,
                1.0,
            ),
            0.01,
        )
        .unwrap();
        assert!(second.state.longitudinal_slip_ratio > first.state.longitudinal_slip_ratio);

        let lifted =
            evaluate_combined_slip_tire(spec, second.state, test_tire_input(None, 0.0, 1.0), 0.01)
                .unwrap();
        assert_eq!(lifted, zero_tire_evaluation());
    }

    #[test]
    fn tire_wrench_preserves_patch_point_and_world_axes() {
        let mut world = World::new();
        let wheel = world.spawn_empty().id();
        let patch = WheelContactPatch {
            point_world_m: Vec3::new(1.0, 2.0, 3.0),
            ..test_patch(wheel, Vec3::ZERO, 100.0)
        };
        let evaluation = CombinedSlipTireEvaluation {
            longitudinal_force_n: 20.0,
            lateral_force_n: -5.0,
            ..zero_tire_evaluation()
        };
        let wrench = combined_slip_tire_wrench(patch, evaluation, Vec3::X, Vec3::Z).unwrap();
        assert_eq!(wrench.entity, wheel);
        assert_eq!(wrench.point_world_m, patch.point_world_m);
        assert_eq!(wrench.force_world_n, Vec3::new(20.0, 0.0, -5.0));
    }
}
