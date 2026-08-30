//! Robot entity components.

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Stable robot identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RobotId(pub Uuid);

impl RobotId {
    /// Creates a new random robot id.
    pub fn new_v4() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for RobotId {
    fn default() -> Self {
        Self::new_v4()
    }
}

/// Top-level robot entity marker.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Robot {
    /// Stable robot identifier.
    pub robot_id: RobotId,
    /// Human-readable model name.
    pub model_name: String,
    /// Base link entity.
    pub base_link: Entity,
}

/// Physical link on a robot.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Link {
    /// Owning robot entity.
    pub robot: Entity,
    /// Link name.
    pub name: String,
}

/// Joint type between two links.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointKind {
    /// Fixed joint with no degrees of freedom.
    Fixed,
    /// Revolute joint about one axis.
    Revolute,
    /// Continuous revolute joint without limits.
    Continuous,
    /// Prismatic joint sliding along one axis.
    Prismatic,
}

/// Joint limit specification.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointLimits {
    /// Lower position limit in radians or meters.
    pub lower: f64,
    /// Upper position limit in radians or meters.
    pub upper: f64,
    /// Maximum velocity in radians per second or meters per second.
    pub max_velocity: f64,
    /// Maximum effort in newton-meters or newtons.
    pub max_effort: f64,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            lower: -f64::INFINITY,
            upper: f64::INFINITY,
            max_velocity: f64::INFINITY,
            max_effort: f64::INFINITY,
        }
    }
}

/// Joint connecting parent and child links.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Joint {
    /// Owning robot entity.
    pub robot: Entity,
    /// Parent link entity.
    pub parent_link: Entity,
    /// Child link entity.
    pub child_link: Entity,
    /// Joint type.
    pub kind: JointKind,
    /// Joint limits.
    pub limits: JointLimits,
    /// Joint axis in parent frame.
    pub axis: Vec3,
    /// Current joint position in radians or meters.
    pub position: f64,
    /// Current joint velocity.
    pub velocity: f64,
}

/// Actuator driving a joint or wheel.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Actuator {
    /// Owning robot entity.
    pub robot: Entity,
    /// Driven joint entity, if any.
    pub joint: Option<Entity>,
    /// Actuator name.
    pub name: String,
    /// Current control mode.
    pub mode: crate::actuator::ControlMode,
    /// Current command target.
    pub target: crate::actuator::ActuatorTarget,
    /// Safety and saturation limits.
    pub limits: crate::actuator::ActuatorLimits,
}

/// Electrical failure applied to a DC motor model.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DcMotorFailureMode {
    /// Normal voltage-driven operation.
    #[default]
    Nominal,
    /// Disconnected terminals: no armature current or electromagnetic torque.
    OpenCircuit,
    /// Shorted terminals: command voltage is zero and back-EMF produces bounded braking current.
    ShortCircuit,
}

/// Identifiable brushed or brushless-DC equivalent-circuit parameters.
///
/// The default fidelity tier is quasi-static: set [`Self::inductance_h`] to `None` and
/// identify resistance, motor constants, losses, and current limits from a datasheet,
/// locked-rotor test, free-spin test, and coast-down trace. Supplying an inductance enables
/// explicit forward-Euler current dynamics and therefore requires a timestep small enough
/// for the electrical time constant. Thermal state, commutation ripple, saturation, and
/// cogging are outside this v1 model and must not be inferred from its output.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DcMotorSpec {
    /// Terminal-to-terminal armature resistance in ohms.
    pub resistance_ohm: f64,
    /// Torque constant in newton-meters per ampere.
    pub torque_constant_nm_a: f64,
    /// Back-EMF constant in volt-seconds per radian.
    pub back_emf_constant_v_s_rad: f64,
    /// Maximum absolute command voltage at the motor terminals in volts.
    pub supply_voltage_v: f64,
    /// Maximum absolute armature current in amperes.
    pub current_limit_a: f64,
    /// Rotor inertia about the shaft in kilogram square meters.
    pub rotor_inertia_kg_m2: f64,
    /// Viscous shaft-loss coefficient in newton-meter-seconds per radian.
    pub viscous_friction_nm_s_rad: f64,
    /// Coulomb shaft-loss magnitude in newton-meters.
    pub coulomb_friction_nm: f64,
    /// Optional armature inductance in henries; `None` selects the quasi-static tier.
    pub inductance_h: Option<f64>,
    /// Explicit electrical failure behavior.
    pub failure_mode: DcMotorFailureMode,
}

impl Default for DcMotorSpec {
    fn default() -> Self {
        Self {
            resistance_ohm: 0.5,
            torque_constant_nm_a: 0.08,
            back_emf_constant_v_s_rad: 0.08,
            supply_voltage_v: 24.0,
            current_limit_a: 20.0,
            rotor_inertia_kg_m2: 0.000_1,
            viscous_friction_nm_s_rad: 0.001,
            coulomb_friction_nm: 0.01,
            inductance_h: None,
            failure_mode: DcMotorFailureMode::Nominal,
        }
    }
}

impl DcMotorSpec {
    /// Returns whether all parameters are finite and physically valid for evaluation.
    pub fn is_valid(&self) -> bool {
        [
            self.resistance_ohm,
            self.torque_constant_nm_a,
            self.back_emf_constant_v_s_rad,
            self.supply_voltage_v,
            self.current_limit_a,
            self.rotor_inertia_kg_m2,
            self.viscous_friction_nm_s_rad,
            self.coulomb_friction_nm,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.resistance_ohm > 0.0
            && self.torque_constant_nm_a > 0.0
            && self.back_emf_constant_v_s_rad > 0.0
            && self.supply_voltage_v > 0.0
            && self.current_limit_a > 0.0
            && self.rotor_inertia_kg_m2 >= 0.0
            && self.viscous_friction_nm_s_rad >= 0.0
            && self.coulomb_friction_nm >= 0.0
            && self
                .inductance_h
                .is_none_or(|inductance_h| inductance_h.is_finite() && inductance_h > 0.0)
    }
}

/// Dynamic electrical state retained by a DC motor evaluator.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DcMotorState {
    /// Armature current at the end of the latest completed step in amperes.
    pub current_a: f64,
}

/// Completed motor electrical telemetry made available to measurement frontends.
///
/// This component contains realized plant outputs, never command targets. A motor
/// integration system may attach or replace it after each completed electrical
/// step. Winding temperature remains `None` until a declared thermal plant exists.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DcMotorCompletedTelemetry {
    /// Realized terminal voltage after supply and failure limits, in volts.
    pub terminal_voltage_v: f64,
    /// Realized armature or torque-producing current, in amperes.
    pub current_a: f64,
    /// Back electromotive force at the completed rotor speed, in volts.
    pub back_emf_v: f64,
    /// Optional completed winding temperature in degrees Celsius.
    pub winding_temperature_c: Option<f64>,
    /// Whether the plant clipped requested terminal voltage.
    pub voltage_saturated: bool,
    /// Whether the plant clipped unconstrained current.
    pub current_saturated: bool,
    /// Electrical failure active during the completed step.
    pub failure_mode: DcMotorFailureMode,
}

/// Backend-neutral motor-to-wheel transmission parameters.
///
/// Positive ratio preserves coordinate sign and negative ratio reverses it. Backlash and
/// torsional compliance are declared here for later stateful driveline integration; the M1-A
/// static torque map reports them but does not pretend to simulate their transient response.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TransmissionSpec {
    /// Motor radians per wheel radian, including coordinate direction.
    pub ratio_motor_rad_per_wheel_rad: f64,
    /// Efficiency ratio when mechanical power flows from motor to wheel, in `[0, 1]`.
    pub drive_efficiency_ratio: f64,
    /// Efficiency ratio when mechanical power flows from wheel to motor, in `[0, 1]`.
    pub backdrive_efficiency_ratio: f64,
    /// Total wheel-side angular backlash in radians.
    pub backlash_rad: f64,
    /// Optional wheel-side torsional stiffness in newton-meters per radian.
    pub torsional_stiffness_nm_rad: Option<f64>,
    /// Wheel-side torsional damping in newton-meter-seconds per radian.
    pub torsional_damping_nm_s_rad: f64,
}

impl Default for TransmissionSpec {
    fn default() -> Self {
        Self {
            ratio_motor_rad_per_wheel_rad: 20.0,
            drive_efficiency_ratio: 0.9,
            backdrive_efficiency_ratio: 0.75,
            backlash_rad: 0.0,
            torsional_stiffness_nm_rad: None,
            torsional_damping_nm_s_rad: 0.0,
        }
    }
}

impl TransmissionSpec {
    /// Returns whether all parameters are finite and physically valid.
    pub fn is_valid(&self) -> bool {
        self.ratio_motor_rad_per_wheel_rad.is_finite()
            && self.ratio_motor_rad_per_wheel_rad != 0.0
            && self.drive_efficiency_ratio.is_finite()
            && (0.0..=1.0).contains(&self.drive_efficiency_ratio)
            && self.backdrive_efficiency_ratio.is_finite()
            && (0.0..=1.0).contains(&self.backdrive_efficiency_ratio)
            && self.backlash_rad.is_finite()
            && self.backlash_rad >= 0.0
            && self
                .torsional_stiffness_nm_rad
                .is_none_or(|stiffness_nm_rad| {
                    stiffness_nm_rad.is_finite() && stiffness_nm_rad > 0.0
                })
            && self.torsional_damping_nm_s_rad.is_finite()
            && self.torsional_damping_nm_s_rad >= 0.0
    }
}

/// Geometry and inertia for one steerable or fixed wheel assembly.
///
/// `forward_axis` and `axle_axis` form the wheel contact frame in the wheel link's local
/// coordinates. They must be unit length and orthogonal. Surface and tire-force laws are
/// intentionally not embedded here so physics backends can share the same assembly spec.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WheelAssemblySpec {
    /// Unloaded rolling radius in meters.
    pub radius_m: f64,
    /// Physical tread width in meters.
    pub width_m: f64,
    /// Wheel rotational inertia about its axle in kilogram square meters.
    pub inertia_kg_m2: f64,
    /// Dimensionless rolling-resistance coefficient.
    pub rolling_resistance_coefficient: f64,
    /// Local unit vector pointing in the free-rolling direction.
    pub forward_axis: Vec3,
    /// Local unit vector along the positive wheel axle.
    pub axle_axis: Vec3,
}

impl Default for WheelAssemblySpec {
    fn default() -> Self {
        Self {
            radius_m: 0.1,
            width_m: 0.04,
            inertia_kg_m2: 0.01,
            rolling_resistance_coefficient: 0.015,
            forward_axis: Vec3::X,
            axle_axis: Vec3::Z,
        }
    }
}

impl WheelAssemblySpec {
    /// Returns whether dimensions, inertia, and the declared contact frame are valid.
    pub fn is_valid(&self) -> bool {
        const AXIS_TOLERANCE: f64 = 1.0e-6;
        [
            self.radius_m,
            self.width_m,
            self.inertia_kg_m2,
            self.rolling_resistance_coefficient,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.radius_m > 0.0
            && self.width_m > 0.0
            && self.inertia_kg_m2 >= 0.0
            && self.rolling_resistance_coefficient >= 0.0
            && self.forward_axis.is_finite()
            && self.axle_axis.is_finite()
            && (self.forward_axis.length() - 1.0).abs() <= AXIS_TOLERANCE
            && (self.axle_axis.length() - 1.0).abs() <= AXIS_TOLERANCE
            && self.forward_axis.dot(self.axle_axis).abs() <= AXIS_TOLERANCE
    }
}

/// Identifiable low-order combined-slip tire parameters.
///
/// This is a force-element model, not a generic collider material. Longitudinal
/// and lateral small-slip stiffnesses are identified at [`Self::reference_load_n`].
/// Peak friction decreases linearly with normalized load, bounded by
/// [`Self::minimum_friction_ratio`], and the resulting uncoupled forces share one
/// smooth friction ellipse. Relaxation lengths retain transient tread response.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CombinedSlipTireSpec {
    /// Reference normal load used for parameter identification, in newtons.
    pub reference_load_n: f64,
    /// Longitudinal small-slip stiffness at reference load, in newtons.
    pub longitudinal_stiffness_n: f64,
    /// Lateral small-slip stiffness at reference load, in newtons per unit tangent of slip angle.
    pub lateral_stiffness_n: f64,
    /// Peak longitudinal friction coefficient at reference load.
    pub longitudinal_peak_friction: f64,
    /// Peak lateral friction coefficient at reference load.
    pub lateral_peak_friction: f64,
    /// Fractional friction loss per unit increase in `load / reference_load`.
    pub load_sensitivity_per_load_ratio: f64,
    /// Lower bound on load-sensitive friction as a ratio of reference friction.
    pub minimum_friction_ratio: f64,
    /// Maximum `load / reference_load` admitted by this model's validity envelope.
    pub maximum_load_ratio: f64,
    /// Positive speed used to regularize slip coordinates near standstill, in meters per second.
    pub low_speed_regularization_m_s: f64,
    /// Longitudinal relaxation length in meters; zero selects instantaneous response.
    pub longitudinal_relaxation_length_m: f64,
    /// Lateral relaxation length in meters; zero selects instantaneous response.
    pub lateral_relaxation_length_m: f64,
}

impl Default for CombinedSlipTireSpec {
    fn default() -> Self {
        Self {
            reference_load_n: 1_000.0,
            longitudinal_stiffness_n: 8_000.0,
            lateral_stiffness_n: 7_000.0,
            longitudinal_peak_friction: 0.9,
            lateral_peak_friction: 0.9,
            load_sensitivity_per_load_ratio: 0.1,
            minimum_friction_ratio: 0.5,
            maximum_load_ratio: 3.0,
            low_speed_regularization_m_s: 0.1,
            longitudinal_relaxation_length_m: 0.05,
            lateral_relaxation_length_m: 0.08,
        }
    }
}

impl CombinedSlipTireSpec {
    /// Returns whether all parameters are finite and inside the declared physical domain.
    pub fn is_valid(&self) -> bool {
        [
            self.reference_load_n,
            self.longitudinal_stiffness_n,
            self.lateral_stiffness_n,
            self.longitudinal_peak_friction,
            self.lateral_peak_friction,
            self.load_sensitivity_per_load_ratio,
            self.minimum_friction_ratio,
            self.maximum_load_ratio,
            self.low_speed_regularization_m_s,
            self.longitudinal_relaxation_length_m,
            self.lateral_relaxation_length_m,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.reference_load_n > 0.0
            && self.longitudinal_stiffness_n > 0.0
            && self.lateral_stiffness_n > 0.0
            && self.longitudinal_peak_friction > 0.0
            && self.lateral_peak_friction > 0.0
            && (0.0..1.0).contains(&self.load_sensitivity_per_load_ratio)
            && (0.0..=1.0).contains(&self.minimum_friction_ratio)
            && self.minimum_friction_ratio > 0.0
            && self.maximum_load_ratio >= 1.0
            && self.low_speed_regularization_m_s > 0.0
            && self.longitudinal_relaxation_length_m >= 0.0
            && self.lateral_relaxation_length_m >= 0.0
    }
}

/// Relaxed combined-slip coordinates retained between completed tire steps.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CombinedSlipTireState {
    /// Relaxed longitudinal slip ratio, positive for driving traction.
    pub longitudinal_slip_ratio: f64,
    /// Relaxed tangent of lateral slip angle, signed in the wheel lateral frame.
    pub lateral_slip_tangent: f64,
}

/// Parameters for a coupled one-dimensional motor-to-road mobility plant.
///
/// One representative driven wheel is simulated and its longitudinal tire
/// force is multiplied by `driven_wheel_count` at the chassis. The declared
/// motor, transmission, inertia, load, and tire state therefore represent one
/// path shared by identical driven wheels. This is a control-oriented
/// straight-line model, not an Ackermann or suspension replacement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalMobilityPlantSpec {
    /// Total translating vehicle mass in kilograms.
    pub vehicle_mass_kg: f64,
    /// Number of identical driven wheel/motor paths.
    pub driven_wheel_count: u32,
    /// Completed normal load carried by each driven wheel in newtons.
    pub normal_load_per_driven_wheel_n: f64,
    /// Constant road grade, positive uphill, in radians.
    pub road_grade_rad: f64,
    /// Quadratic aerodynamic force coefficient in newton-seconds squared per square meter.
    pub aerodynamic_drag_n_s2_m2: f64,
    /// Road friction multiplier applied to the tire model.
    pub road_friction_scale: f64,
    /// Electrical motor model for each driven wheel.
    pub motor: DcMotorSpec,
    /// Motor-to-wheel transmission for each driven wheel.
    pub transmission: TransmissionSpec,
    /// Driven wheel geometry, inertia, and rolling resistance.
    pub wheel: WheelAssemblySpec,
    /// Transient combined-slip tire model; lateral input remains zero in this plant.
    pub tire: CombinedSlipTireSpec,
}

impl LongitudinalMobilityPlantSpec {
    /// Returns whether all physical parameters and nested plant specs are valid.
    pub fn is_valid(self) -> bool {
        self.vehicle_mass_kg.is_finite()
            && self.vehicle_mass_kg > 0.0
            && self.driven_wheel_count > 0
            && self.normal_load_per_driven_wheel_n.is_finite()
            && self.normal_load_per_driven_wheel_n > 0.0
            && self.road_grade_rad.is_finite()
            && self.road_grade_rad.abs() < std::f64::consts::FRAC_PI_2
            && self.aerodynamic_drag_n_s2_m2.is_finite()
            && self.aerodynamic_drag_n_s2_m2 >= 0.0
            && self.road_friction_scale.is_finite()
            && self.road_friction_scale >= 0.0
            && self.motor.is_valid()
            && self.transmission.is_valid()
            && self.wheel.is_valid()
            && self.tire.is_valid()
    }
}

/// Dynamic state of [`LongitudinalMobilityPlantSpec`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LongitudinalMobilityPlantState {
    /// Chassis longitudinal position in meters.
    pub position_m: f64,
    /// Chassis longitudinal velocity in meters per second.
    pub velocity_m_s: f64,
    /// Representative driven-wheel angle in radians.
    pub wheel_position_rad: f64,
    /// Representative driven-wheel angular velocity in radians per second.
    pub wheel_velocity_rad_s: f64,
    /// Electrical current state for the representative motor.
    pub motor_state: DcMotorState,
    /// Relaxed tire-slip state for the representative tire.
    pub tire_state: CombinedSlipTireState,
}

/// Deterministic kinematic Ackermann drive state and safety limits.
///
/// The drive uses the entity's local `+X` axis as its forward direction. Commands
/// are clamped to the configured speed and steering limits. The integration
/// system ignores an entity when its limits are non-finite or physically invalid.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct AckermannDrive {
    /// Distance between front and rear axles in meters.
    pub wheelbase_m: f64,
    /// Maximum absolute forward or reverse speed in meters per second.
    pub max_speed_m_s: f64,
    /// Maximum absolute front-wheel steering angle in radians.
    pub max_steering_rad: f64,
    /// Maximum speed increase per second in meters per second squared.
    pub max_acceleration_m_s2: f64,
    /// Maximum braking or direction-change rate in meters per second squared.
    pub max_deceleration_m_s2: f64,
    /// Maximum steering-angle change in radians per second.
    pub max_steering_rate_rad_s: f64,
    /// Current signed longitudinal speed in meters per second.
    pub speed_m_s: f64,
    /// Current front-wheel steering angle in radians.
    pub steering_rad: f64,
    /// Clamped target signed longitudinal speed in meters per second.
    pub target_speed_m_s: f64,
    /// Clamped target front-wheel steering angle in radians.
    pub target_steering_rad: f64,
}

impl Default for AckermannDrive {
    fn default() -> Self {
        Self {
            wheelbase_m: 2.7,
            max_speed_m_s: 13.9,
            max_steering_rad: 0.6,
            max_acceleration_m_s2: 2.5,
            max_deceleration_m_s2: 5.0,
            max_steering_rate_rad_s: 0.8,
            speed_m_s: 0.0,
            steering_rad: 0.0,
            target_speed_m_s: 0.0,
            target_steering_rad: 0.0,
        }
    }
}

impl AckermannDrive {
    /// Returns whether all limits and state values are finite and physically valid.
    pub fn is_valid(&self) -> bool {
        [
            self.wheelbase_m,
            self.max_speed_m_s,
            self.max_steering_rad,
            self.max_acceleration_m_s2,
            self.max_deceleration_m_s2,
            self.max_steering_rate_rad_s,
            self.speed_m_s,
            self.steering_rad,
            self.target_speed_m_s,
            self.target_steering_rad,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.wheelbase_m > 0.0
            && self.max_speed_m_s >= 0.0
            && self.max_steering_rad >= 0.0
            && self.max_acceleration_m_s2 >= 0.0
            && self.max_deceleration_m_s2 >= 0.0
            && self.max_steering_rate_rad_s >= 0.0
    }
}

/// Deterministic multirotor position-flight state and safety limits.
///
/// The controller uses the entity's [`rne_world::Transform3`] as the aircraft
/// pose in a Y-up world. Position targets are converted into bounded velocity
/// and acceleration commands. Horizontal acceleration tilts the rendered body,
/// while climb speed, yaw rate, and total acceleration remain independently
/// limited. Invalid configurations are ignored transactionally by
/// [`crate::multirotor_flight`].
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MultirotorFlight {
    /// Maximum horizontal speed in meters per second.
    pub max_horizontal_speed_m_s: f64,
    /// Maximum absolute climb or descent speed in meters per second.
    pub max_climb_speed_m_s: f64,
    /// Maximum translational acceleration magnitude in meters per second squared.
    pub max_acceleration_m_s2: f64,
    /// Maximum yaw rate in radians per second.
    pub max_yaw_rate_rad_s: f64,
    /// Maximum body tilt from world up in radians.
    pub max_tilt_rad: f64,
    /// Position-error gain in inverse seconds.
    pub position_gain_s_inv: f64,
    /// Velocity-error gain in inverse seconds.
    pub velocity_gain_s_inv: f64,
    /// First-order rendered-attitude response time in seconds.
    pub attitude_response_s: f64,
    /// Current world-space velocity in meters per second.
    pub velocity_m_s: Vec3,
    /// Current integrated yaw target in radians.
    pub yaw_rad: f64,
    /// Requested world-space position in meters.
    pub target_position_m: Vec3,
    /// Requested world-space heading in radians.
    pub target_yaw_rad: f64,
    /// Bounded acceleration applied by the most recent simulation step.
    pub commanded_acceleration_m_s2: Vec3,
}

impl Default for MultirotorFlight {
    fn default() -> Self {
        Self {
            max_horizontal_speed_m_s: 12.0,
            max_climb_speed_m_s: 4.0,
            max_acceleration_m_s2: 6.0,
            max_yaw_rate_rad_s: 1.2,
            max_tilt_rad: 0.52,
            position_gain_s_inv: 0.8,
            velocity_gain_s_inv: 2.5,
            attitude_response_s: 0.12,
            velocity_m_s: Vec3::ZERO,
            yaw_rad: 0.0,
            target_position_m: Vec3::ZERO,
            target_yaw_rad: 0.0,
            commanded_acceleration_m_s2: Vec3::ZERO,
        }
    }
}

impl MultirotorFlight {
    /// Returns whether every limit, gain, command, and state value is finite and valid.
    pub fn is_valid(&self) -> bool {
        [
            self.max_horizontal_speed_m_s,
            self.max_climb_speed_m_s,
            self.max_acceleration_m_s2,
            self.max_yaw_rate_rad_s,
            self.max_tilt_rad,
            self.position_gain_s_inv,
            self.velocity_gain_s_inv,
            self.attitude_response_s,
            self.yaw_rad,
            self.target_yaw_rad,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.velocity_m_s.is_finite()
            && self.target_position_m.is_finite()
            && self.commanded_acceleration_m_s2.is_finite()
            && self.max_horizontal_speed_m_s >= 0.0
            && self.max_climb_speed_m_s >= 0.0
            && self.max_acceleration_m_s2 >= 0.0
            && self.max_yaw_rate_rad_s >= 0.0
            && (0.0..std::f64::consts::FRAC_PI_2).contains(&self.max_tilt_rad)
            && self.position_gain_s_inv > 0.0
            && self.velocity_gain_s_inv > 0.0
            && self.attitude_response_s >= 0.0
    }
}

/// Inertial properties for a link.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Inertial {
    /// Mass in kilograms.
    pub mass_kg: f64,
    /// Center of mass offset in meters.
    pub center_of_mass_m: Vec3,
    /// Orientation of inertial frame.
    pub orientation: Quat,
}

impl Default for Inertial {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            center_of_mass_m: Vec3::ZERO,
            orientation: Quat::IDENTITY,
        }
    }
}

/// Planar dynamic bicycle model state and parameters for an Ackermann vehicle.
///
/// [`crate::ackermann_kinematics`] assumes the tires never slip, which makes every
/// controller look perfect: the vehicle goes exactly where the steering points it.
/// Attaching this component opts a vehicle into the single-track *dynamic* model
/// instead, where lateral tire forces are finite. Understeer, oversteer, and the
/// widening of a line with speed all emerge from the force balance rather than being
/// scripted.
///
/// The model runs in the ground plane. Front and rear slip angles produce lateral
/// forces through a linear tire that saturates at the friction limit, and longitudinal
/// weight transfer shifts that limit between the axles under acceleration and braking.
/// Below [`Self::blend_low_speed_m_s`] the update blends into the kinematic solution,
/// because slip angles divide by forward speed and become singular near standstill.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VehicleDynamics {
    /// Vehicle mass in kilograms.
    pub mass_kg: f64,
    /// Yaw moment of inertia in kilogram square meters.
    pub yaw_inertia_kg_m2: f64,
    /// Distance from the center of mass to the front axle in meters.
    pub front_axle_m: f64,
    /// Distance from the center of mass to the rear axle in meters.
    pub rear_axle_m: f64,
    /// Height of the center of mass above ground in meters, for load transfer.
    pub center_of_mass_height_m: f64,
    /// Front axle cornering stiffness in newtons per radian.
    pub front_cornering_stiffness_n_rad: f64,
    /// Rear axle cornering stiffness in newtons per radian.
    pub rear_cornering_stiffness_n_rad: f64,
    /// Tire-road friction coefficient.
    pub friction_coefficient: f64,
    /// Forward speed below which the kinematic solution takes over, in meters per second.
    pub blend_low_speed_m_s: f64,
    /// First-order steering actuator time constant in seconds; `0.0` is instantaneous.
    ///
    /// A real steering actuator does not reach its target within one control tick: the
    /// steering column follows the command with a lag. This delays the whole lateral
    /// response, which is exactly the phase loss that destabilizes aggressively tuned
    /// controllers on hardware while they look fine against an instant plant.
    pub steering_lag_s: f64,
    /// Current lateral velocity at the center of mass in meters per second.
    pub lateral_velocity_m_s: f64,
    /// Current yaw rate in radians per second.
    pub yaw_rate_rad_s: f64,
    /// Front slip angle of the last step in radians, for telemetry.
    pub front_slip_rad: f64,
    /// Rear slip angle of the last step in radians, for telemetry.
    pub rear_slip_rad: f64,
    /// Whether the front axle saturated its friction limit during the last step.
    pub front_saturated: bool,
    /// Whether the rear axle saturated its friction limit during the last step.
    pub rear_saturated: bool,
}

impl Default for VehicleDynamics {
    fn default() -> Self {
        Self {
            // A mid-size sedan; cornering stiffness values are per axle.
            mass_kg: 1_500.0,
            yaw_inertia_kg_m2: 2_250.0,
            front_axle_m: 1.2,
            rear_axle_m: 1.5,
            center_of_mass_height_m: 0.55,
            front_cornering_stiffness_n_rad: 80_000.0,
            rear_cornering_stiffness_n_rad: 88_000.0,
            friction_coefficient: 0.9,
            blend_low_speed_m_s: 2.0,
            steering_lag_s: 0.0,
            lateral_velocity_m_s: 0.0,
            yaw_rate_rad_s: 0.0,
            front_slip_rad: 0.0,
            rear_slip_rad: 0.0,
            front_saturated: false,
            rear_saturated: false,
        }
    }
}

impl VehicleDynamics {
    /// Returns whether all parameters are finite and physically valid.
    pub fn is_valid(&self) -> bool {
        [
            self.mass_kg,
            self.yaw_inertia_kg_m2,
            self.front_axle_m,
            self.rear_axle_m,
            self.center_of_mass_height_m,
            self.front_cornering_stiffness_n_rad,
            self.rear_cornering_stiffness_n_rad,
            self.friction_coefficient,
            self.blend_low_speed_m_s,
            self.lateral_velocity_m_s,
            self.yaw_rate_rad_s,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.mass_kg > 0.0
            && self.yaw_inertia_kg_m2 > 0.0
            && self.front_axle_m > 0.0
            && self.rear_axle_m > 0.0
            && self.center_of_mass_height_m >= 0.0
            && self.front_cornering_stiffness_n_rad > 0.0
            && self.rear_cornering_stiffness_n_rad > 0.0
            && self.friction_coefficient > 0.0
            && self.blend_low_speed_m_s >= 0.0
            && self.steering_lag_s.is_finite()
            && self.steering_lag_s >= 0.0
    }

    /// Wheelbase implied by the axle distances, in meters.
    pub fn wheelbase_m(&self) -> f64 {
        self.front_axle_m + self.rear_axle_m
    }

    /// Static front axle load in newtons under standard gravity.
    pub fn static_front_load_n(&self) -> f64 {
        self.mass_kg * 9.81 * self.rear_axle_m / self.wheelbase_m()
    }

    /// Static rear axle load in newtons under standard gravity.
    pub fn static_rear_load_n(&self) -> f64 {
        self.mass_kg * 9.81 * self.front_axle_m / self.wheelbase_m()
    }
}
