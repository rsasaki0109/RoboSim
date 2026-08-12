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
