//! Explicit four-wheel Ackermann suspension benchmark shared by rigid-body backends.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, Collider, CollisionGroups, ExternalBodyWrench, JointActuation,
    JointMotorGainModel, JointState, MultibodyLink, PhysicsBackend, PhysicsBackendManifest,
    PhysicsCapability, PhysicsMaterial, PhysicsWorldDesc, PrismaticJointDesc, RevoluteJointDesc,
    RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, evaluate_steering_actuator,
    evaluate_suspension_strut, DcMotorSpec, LongitudinalDrivePathInput, LongitudinalDrivePathState,
    LongitudinalMobilityPlantSpec, SteeringActuatorSpec, SteeringActuatorState,
    SuspensionStrutSpec, TransmissionSpec, WheelAssemblySpec, WheelStationSpec,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

use crate::MobilityBenchmarkMetric;

/// Stable trace kind for the explicit Ackermann suspension task.
pub const ACKERMANN_SUSPENSION_TRACE_KIND: &str = "rne_mobility_ackermann_suspension_trace";
/// Stable comparison kind for two backend traces.
pub const ACKERMANN_SUSPENSION_COMPARISON_KIND: &str =
    "rne_mobility_ackermann_suspension_comparison";
/// Schema version for the trace and comparison.
pub const ACKERMANN_SUSPENSION_SCHEMA_VERSION: u32 = 2;
/// Stable task identity shared by both backends.
pub const ACKERMANN_SUSPENSION_TASK_ID: &str = "mobility_ackermann_suspension_split_mu_v2";
/// One-millisecond fixed physics step in simulation ticks.
pub const ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SETTLE_STEPS: u64 = 1_500;
const ACCEL_END_STEP: u64 = 3_500;
const TURN_END_STEP: u64 = 5_500;
const TOTAL_STEPS: u64 = 7_000;
const TRACE_STRIDE_STEPS: u64 = 100;
const WORLD_SEED: u64 = 0;
pub(crate) const CHASSIS_MASS_KG: f64 = 600.0;
pub(crate) const WHEEL_RADIUS_M: f64 = 0.28;
pub(crate) const WHEELBASE_M: f64 = 2.20;
pub(crate) const TRACK_WIDTH_M: f64 = 1.30;
pub(crate) const INITIAL_SUSPENSION_POSITION_M: f64 = -0.052;
const STEERING_COMMAND_RAD: f64 = 0.35;
const DRIVE_VOLTAGE_V: f64 = 6.0;
pub(crate) const CONTACT_LOAD_FILTER_TIME_CONSTANT_S: f64 = 0.02;
const SUSPENSION_LIMIT_SOLVER_TOLERANCE_M: f64 = 0.001;
pub(crate) const SELF_COLLISION_GROUP: u32 = 2;

/// One sampled state from the four-wheel multibody experiment.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AckermannSuspensionSample {
    /// Completed physics step.
    pub step: u64,
    /// Completed simulation time in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Maneuver phase: settle, accelerate, turn, or brake.
    pub phase: u8,
    /// Requested center steering coordinate in radians.
    pub steering_command_rad: f64,
    /// Completed center steering-actuator target sent through Ackermann geometry, in radians.
    pub steering_actuator_target_rad: f64,
    /// Actual front-left/front-right steering coordinates in radians.
    pub front_steering_rad: [f64; 2],
    /// Commanded terminal voltage for each wheel in FL, RL, FR, RR order.
    pub command_voltage_v: [f64; 4],
    /// Privileged chassis position in world meters.
    pub privileged_position_world_m: [f64; 3],
    /// Privileged chassis linear velocity in world meters per second.
    pub privileged_linear_velocity_world_m_s: [f64; 3],
    /// Privileged chassis angular velocity in world radians per second.
    pub privileged_angular_velocity_world_rad_s: [f64; 3],
    /// Completed suspension coordinates in meters.
    pub suspension_position_m: [f64; 4],
    /// Completed suspension velocities in meters per second.
    pub suspension_velocity_m_s: [f64; 4],
    /// Conditioned solved normal load at each wheel in newtons.
    pub wheel_normal_load_n: [f64; 4],
    /// Independent wheel angular velocities in radians per second.
    pub wheel_velocity_rad_s: [f64; 4],
    /// Tire longitudinal forces in newtons.
    pub wheel_longitudinal_force_n: [f64; 4],
    /// Tire lateral forces in newtons.
    pub wheel_lateral_force_n: [f64; 4],
    /// Combined-friction utilization for each wheel.
    pub wheel_friction_utilization: [f64; 4],
}

impl AckermannSuspensionSample {
    fn validate(&self) -> Result<()> {
        ensure!(self.step <= TOTAL_STEPS, "sample step outside task");
        ensure!(
            self.sim_time_ticks == self.step * ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
            "sample time drift"
        );
        ensure!(self.phase == phase_for_step(self.step), "phase drift");
        ensure!(
            self.steering_command_rad == command_for_step(self.step).0,
            "steering command drift"
        );
        ensure!(
            self.steering_actuator_target_rad.abs()
                <= steering_actuator_spec().maximum_position_rad,
            "steering actuator target outside travel"
        );
        ensure!(
            self.command_voltage_v == command_for_step(self.step).1,
            "voltage command drift"
        );
        ensure!(
            self.front_steering_rad
                .iter()
                .chain(self.command_voltage_v.iter())
                .chain(self.privileged_position_world_m.iter())
                .chain(self.privileged_linear_velocity_world_m_s.iter())
                .chain(self.privileged_angular_velocity_world_rad_s.iter())
                .chain(self.suspension_position_m.iter())
                .chain(self.suspension_velocity_m_s.iter())
                .chain(self.wheel_normal_load_n.iter())
                .chain(self.wheel_velocity_rad_s.iter())
                .chain(self.wheel_longitudinal_force_n.iter())
                .chain(self.wheel_lateral_force_n.iter())
                .chain(self.wheel_friction_utilization.iter())
                .all(|value| value.is_finite()),
            "non-finite Ackermann sample"
        );
        ensure!(
            self.wheel_normal_load_n.iter().all(|load| *load >= 0.0),
            "negative wheel load"
        );
        ensure!(
            self.wheel_friction_utilization
                .iter()
                .all(|value| (0.0..=1.0 + 1.0e-12).contains(value)),
            "friction utilization outside ellipse"
        );
        Ok(())
    }
}

/// Self-verifying trace for one backend execution.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AckermannSuspensionTrace {
    /// Artifact kind.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// Exact backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact portable task contract.
    pub task_spec: TaskSpec,
    /// Per-wheel motor, transmission, inertia, and tire contract.
    pub wheel_plant_spec: LongitudinalMobilityPlantSpec,
    /// Ordered FL, RL, FR, RR station geometry.
    pub wheel_station_specs: [WheelStationSpec; 4],
    /// Shared linear suspension force-element contract.
    pub suspension_spec: SuspensionStrutSpec,
    /// Shared center-steering actuator response contract.
    pub steering_actuator_spec: SteeringActuatorSpec,
    /// Wheelbase in meters.
    pub wheelbase_m: f64,
    /// Track width in meters.
    pub track_width_m: f64,
    /// Fixed physics step in simulation ticks.
    pub fixed_delta_ticks: u64,
    /// Deterministic world seed.
    pub seed: u64,
    /// Completed task steps.
    pub steps: u64,
    /// Ordered downsampled states.
    pub samples: Vec<AckermannSuspensionSample>,
    /// Unit-bearing acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Overall verdict recomputed from metrics.
    pub passed: bool,
    /// FNV-1a digest over all preceding fields.
    pub content_digest: String,
}

impl AckermannSuspensionTrace {
    /// Recomputes the frozen contract, metric verdict, sample order, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == ACKERMANN_SUSPENSION_TRACE_KIND, "kind drift");
        ensure!(
            self.schema_version == ACKERMANN_SUSPENSION_SCHEMA_VERSION,
            "schema drift"
        );
        self.backend.validate()?;
        ensure!(
            self.task_spec == ackermann_suspension_task_spec(),
            "task drift"
        );
        ensure!(self.wheel_plant_spec == wheel_plant_spec(), "plant drift");
        ensure!(
            self.wheel_station_specs == wheel_station_specs(),
            "station drift"
        );
        ensure!(
            self.suspension_spec == suspension_spec(),
            "suspension drift"
        );
        ensure!(
            self.steering_actuator_spec == steering_actuator_spec(),
            "steering actuator drift"
        );
        ensure!(self.wheelbase_m == WHEELBASE_M, "wheelbase drift");
        ensure!(self.track_width_m == TRACK_WIDTH_M, "track drift");
        ensure!(
            self.fixed_delta_ticks == ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS
                && self.seed == WORLD_SEED
                && self.steps == TOTAL_STEPS,
            "runtime contract drift"
        );
        ensure!(
            self.samples.first().is_some_and(|sample| sample.step == 0)
                && self
                    .samples
                    .last()
                    .is_some_and(|sample| sample.step == TOTAL_STEPS),
            "trace endpoints missing"
        );
        for pair in self.samples.windows(2) {
            ensure!(pair[0].step < pair[1].step, "sample order drift");
        }
        for sample in &self.samples {
            sample.validate()?;
            for (wheel_index, position_m) in
                sample.suspension_position_m.iter().copied().enumerate()
            {
                ensure!(
                    (self.suspension_spec.minimum_position_m
                        - SUSPENSION_LIMIT_SOLVER_TOLERANCE_M
                        ..=self.suspension_spec.maximum_position_m
                            + SUSPENSION_LIMIT_SOLVER_TOLERANCE_M)
                        .contains(&position_m),
                    "suspension coordinate {position_m} m for wheel {wheel_index} at step {} outside declared travel",
                    sample.step
                );
            }
        }
        let dt_s = SimDuration::from_ticks(self.fixed_delta_ticks)
            .as_seconds()
            .value();
        let mut steering_state = SteeringActuatorState::default();
        let mut sample_index = 1;
        for step in 1..=self.steps {
            steering_state = evaluate_steering_actuator(
                self.steering_actuator_spec,
                steering_state,
                command_for_step(step).0,
                dt_s,
            )?
            .state;
            if sample_index < self.samples.len() && self.samples[sample_index].step == step {
                ensure!(
                    self.samples[sample_index].steering_actuator_target_rad
                        == steering_state.position_rad,
                    "steering actuator replay drift"
                );
                sample_index += 1;
            }
        }
        ensure!(sample_index == self.samples.len(), "unreplayed samples");
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == trace_digest(self)?,
            "trace digest mismatch"
        );
        Ok(())
    }
}

/// Self-verifying SI-unit comparison between two backend traces.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AckermannSuspensionComparison {
    /// Artifact kind.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// First backend trace.
    pub first: AckermannSuspensionTrace,
    /// Second backend trace.
    pub second: AckermannSuspensionTrace,
    /// Unit-bearing cross-backend metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Overall verdict.
    pub passed: bool,
    /// FNV-1a digest over all preceding fields.
    pub content_digest: String,
}

impl AckermannSuspensionComparison {
    /// Recomputes both traces, tolerance metrics, verdict, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == ACKERMANN_SUSPENSION_COMPARISON_KIND,
            "kind drift"
        );
        ensure!(
            self.schema_version == ACKERMANN_SUSPENSION_SCHEMA_VERSION,
            "schema drift"
        );
        self.first.validate()?;
        self.second.validate()?;
        ensure!(
            self.metrics == comparison_metrics(&self.first, &self.second)?,
            "comparison metric drift"
        );
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == comparison_digest(self)?,
            "comparison digest mismatch"
        );
        Ok(())
    }
}

/// Returns the exact portable task executed by the explicit four-wheel plant.
pub fn ackermann_suspension_task_spec() -> TaskSpec {
    TaskSpec::new(
        ACKERMANN_SUSPENSION_TASK_ID,
        ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new(
                "diagnostic_wheel_velocity_rad_s",
                TensorDType::F64,
                vec![4],
                "rad/s",
            ),
            TensorSpec::new(
                "diagnostic_steering_position_rad",
                TensorDType::F64,
                vec![2],
                "rad",
            ),
            TensorSpec::new(
                "diagnostic_suspension_position_m",
                TensorDType::F64,
                vec![4],
                "m",
            ),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new(
                "center_steering_target_rad",
                TensorDType::F64,
                vec![],
                "rad",
            )
            .with_bounds(TensorBounds::broadcast(-0.45, 0.45)),
            TensorSpec::new(
                "four_motor_terminal_voltage_v",
                TensorDType::F64,
                vec![4],
                "V",
            )
            .with_bounds(TensorBounds::broadcast(-24.0, 24.0)),
        ]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_forward_progress_m", 1.0, "m"),
            RewardTermSpec::new("truth_contact_loss", -1.0, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "truth_body_out_of_bounds",
                TerminationKind::Failure,
            )],
            Some(TOTAL_STEPS),
        ),
        ResetSpec::splitmix64(false),
    )
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct StationEntities {
    pub(crate) slider: Entity,
    pub(crate) wheel: Entity,
    pub(crate) spec: WheelStationSpec,
    pub(crate) front: bool,
}

/// Runs the same explicit suspension and tire force elements through one backend.
pub fn run_ackermann_suspension_trace<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<AckermannSuspensionTrace> {
    manifest.validate()?;
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::Articulation,
            PhysicsCapability::ContactForce,
            PhysicsCapability::ContactPointKinematics,
            PhysicsCapability::ExternalBodyWrench,
        ],
    )?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "capability drift"
    );
    let task_spec = ackermann_suspension_task_spec();
    task_spec.validate()?;
    let plant = wheel_plant_spec();
    let suspension = suspension_spec();
    let steering_actuator = steering_actuator_spec();
    ensure!(suspension.is_valid(), "invalid suspension fixture");
    ensure!(
        steering_actuator.is_valid(),
        "invalid steering actuator fixture"
    );
    let fixed_delta = SimDuration::from_ticks(ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 48,
    })?;
    let mut world = World::new();
    let ground = spawn_named(&mut world, "ackermann_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        frictionless_cuboid(Vec3::new(30.0, 0.5, 30.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let chassis = spawn_named(&mut world, "ackermann_chassis");
    world.entity_mut(chassis).insert((
        RigidBody {
            mass_kg: CHASSIS_MASS_KG,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::new(0.0, -0.08, 0.0),
            ixx_kg_m2: 220.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 360.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 500.0,
        },
        MultibodyLink,
        frictionless_cuboid(Vec3::new(1.25, 0.10, 0.72)),
        CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
        Transform3::from_translation_rotation(
            Vec3::new(
                0.0,
                WHEEL_RADIUS_M + 0.15 - INITIAL_SUSPENSION_POSITION_M,
                0.0,
            ),
            Quat::IDENTITY,
        ),
    ));
    let stations = spawn_stations(&mut world, chassis, suspension)?;
    backend.sync_from_ecs(&mut world, physics_world)?;

    let initial_transform = *world
        .get::<Transform3>(chassis)
        .context("initial chassis")?;
    let mut drive_states = [LongitudinalDrivePathState::default(); 4];
    let mut steering_actuator_state = SteeringActuatorState::default();
    let mut conditioned_load_n = [0.0; 4];
    let mut pending_wrenches: Vec<ExternalBodyWrench> = Vec::new();
    let mut contact_steps = [0_u64; 4];
    let mut maximum_abs_steering_rad = 0.0_f64;
    let mut maximum_abs_yaw_rate_rad_s = 0.0_f64;
    let mut maximum_front_rear_load_shift_n = 0.0_f64;
    let mut maximum_left_right_load_difference_n = 0.0_f64;
    let mut maximum_split_mu_utilization_gap = 0.0_f64;
    let mut settled_front_rear_load_difference_n = None;
    let mut settled_left_right_load_difference_n = None;
    let mut suspension_min_m = [f64::INFINITY; 4];
    let mut suspension_max_m = [f64::NEG_INFINITY; 4];
    let mut samples = vec![sample_zero(initial_transform)];

    for zero_based_step in 0..TOTAL_STEPS {
        let step = zero_based_step + 1;
        let (center_steering, command_voltage_v) = command_for_step(step);
        steering_actuator_state = evaluate_steering_actuator(
            steering_actuator,
            steering_actuator_state,
            center_steering,
            dt_s,
        )?
        .state;
        let steering_targets = front_steering_targets(steering_actuator_state.position_rad);
        for (index, station) in stations.iter().enumerate() {
            if station.front {
                let target = if index == 0 {
                    steering_targets[0]
                } else {
                    steering_targets[1]
                };
                world
                    .entity_mut(station.wheel)
                    .insert(JointActuation::RevolutePosition {
                        target_position_rad: target,
                        stiffness_nm_per_rad: 1_500.0,
                        damping_nm_s_per_rad: 120.0,
                        max_effort_nm: 2_000.0,
                    });
            }
        }
        backend.sync_from_ecs(&mut world, physics_world)?;
        for wrench in pending_wrenches.drain(..) {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;

        let transform = *world.get::<Transform3>(chassis).context("chassis pose")?;
        let body = *world.get::<RigidBody>(chassis).context("chassis body")?;
        let contacts = backend.contact_points(physics_world)?;
        let load_alpha = dt_s / (CONTACT_LOAD_FILTER_TIME_CONSTANT_S + dt_s);
        let mut suspension_position_m = [0.0; 4];
        let mut suspension_velocity_m_s = [0.0; 4];
        let mut steering_rad = [0.0; 2];
        let mut wheel_longitudinal_force_n = [0.0; 4];
        let mut wheel_lateral_force_n = [0.0; 4];
        let mut wheel_friction_utilization = [0.0; 4];

        for (index, station) in stations.iter().enumerate() {
            let suspension_state = *world
                .get::<JointState>(station.slider)
                .with_context(|| format!("suspension state {index}"))?;
            let (position_m, velocity_m_s) = match suspension_state {
                JointState::Prismatic {
                    position_m,
                    velocity_m_s,
                } => (position_m, velocity_m_s),
                _ => anyhow::bail!("suspension joint state kind"),
            };
            suspension_position_m[index] = position_m;
            suspension_velocity_m_s[index] = velocity_m_s;
            world
                .entity_mut(station.slider)
                .insert(evaluate_suspension_strut(
                    suspension,
                    position_m,
                    velocity_m_s,
                )?);
            if step > SETTLE_STEPS {
                suspension_min_m[index] = suspension_min_m[index].min(position_m);
                suspension_max_m[index] = suspension_max_m[index].max(position_m);
            }

            if station.front {
                let state = *world
                    .get::<JointState>(station.wheel)
                    .with_context(|| format!("steering state {index}"))?;
                let value = match state {
                    JointState::Revolute { position_rad, .. } => position_rad,
                    _ => anyhow::bail!("steering joint state kind"),
                };
                steering_rad[if index == 0 { 0 } else { 1 }] = value;
                maximum_abs_steering_rad = maximum_abs_steering_rad.max(value.abs());
            }

            let wheel_transform = *world
                .get::<Transform3>(station.wheel)
                .with_context(|| format!("wheel transform {index}"))?;
            let forward_world = (wheel_transform.rotation * Vec3::X).normalize();
            let lateral_world = (wheel_transform.rotation * Vec3::Z).normalize();
            let raw_patch = aggregate_wheel_contact_patch(
                station.wheel,
                contacts,
                forward_world,
                lateral_world,
            )?;
            if step > SETTLE_STEPS && raw_patch.is_some() {
                contact_steps[index] += 1;
            }
            let bounded_load_n = raw_patch
                .map_or(0.0, |patch| patch.normal_load_n)
                .min(plant.tire.reference_load_n * plant.tire.maximum_load_ratio);
            conditioned_load_n[index] += load_alpha * (bounded_load_n - conditioned_load_n[index]);
            let patch = raw_patch.map(|mut patch| {
                patch.normal_load_n = conditioned_load_n[index];
                patch
            });
            let mut station_plant = plant;
            station_plant.road_friction_scale = road_friction_scale(index, step);
            let evaluation = evaluate_longitudinal_drive_path(
                station_plant,
                drive_states[index],
                LongitudinalDrivePathInput {
                    carrier_patch: patch,
                    forward_world,
                    lateral_world,
                    command_voltage_v: command_voltage_v[index],
                },
                dt_s,
            )?;
            drive_states[index] = evaluation.state;
            wheel_longitudinal_force_n[index] = evaluation.tire.longitudinal_force_n;
            wheel_lateral_force_n[index] = evaluation.tire.lateral_force_n;
            wheel_friction_utilization[index] = evaluation.tire.friction_utilization;
            if let Some(wrench) = evaluation.tire_wrench {
                pending_wrenches.push(wrench);
            }
        }

        maximum_abs_yaw_rate_rad_s =
            maximum_abs_yaw_rate_rad_s.max(body.angular_velocity_rad_s.y.abs());
        let front_load_n = conditioned_load_n[0] + conditioned_load_n[2];
        let rear_load_n = conditioned_load_n[1] + conditioned_load_n[3];
        let left_load_n = conditioned_load_n[0] + conditioned_load_n[1];
        let right_load_n = conditioned_load_n[2] + conditioned_load_n[3];
        if step == SETTLE_STEPS {
            settled_front_rear_load_difference_n = Some(front_load_n - rear_load_n);
            settled_left_right_load_difference_n = Some(left_load_n - right_load_n);
        } else if step > SETTLE_STEPS {
            let front_rear_shift_n = (front_load_n
                - rear_load_n
                - settled_front_rear_load_difference_n.context("settled axle loads")?)
            .abs();
            maximum_front_rear_load_shift_n =
                maximum_front_rear_load_shift_n.max(front_rear_shift_n);
            let left_right_shift_n = (left_load_n
                - right_load_n
                - settled_left_right_load_difference_n.context("settled side loads")?)
            .abs();
            maximum_left_right_load_difference_n =
                maximum_left_right_load_difference_n.max(left_right_shift_n);
        }
        if step > TURN_END_STEP {
            let left_utilization =
                0.5 * (wheel_friction_utilization[0] + wheel_friction_utilization[1]);
            let right_utilization =
                0.5 * (wheel_friction_utilization[2] + wheel_friction_utilization[3]);
            maximum_split_mu_utilization_gap =
                maximum_split_mu_utilization_gap.max((left_utilization - right_utilization).abs());
        }

        if step % TRACE_STRIDE_STEPS == 0 || step == TOTAL_STEPS {
            samples.push(make_sample(
                step,
                center_steering,
                steering_actuator_state.position_rad,
                steering_rad,
                command_voltage_v,
                transform,
                body,
                suspension_position_m,
                suspension_velocity_m_s,
                conditioned_load_n,
                drive_states.map(|state| state.wheel_velocity_rad_s),
                wheel_longitudinal_force_n,
                wheel_lateral_force_n,
                wheel_friction_utilization,
            ));
        }
    }

    let final_transform = *world.get::<Transform3>(chassis).context("final chassis")?;
    let displacement_x_m = final_transform.translation.x - initial_transform.translation.x;
    let lateral_displacement_m =
        (final_transform.translation.z - initial_transform.translation.z).abs();
    let driven_duration = TOTAL_STEPS - SETTLE_STEPS;
    let minimum_contact_fraction = contact_steps
        .iter()
        .map(|steps| *steps as f64 / driven_duration as f64)
        .fold(1.0_f64, f64::min);
    let maximum_suspension_travel_range_m = suspension_min_m
        .iter()
        .zip(suspension_max_m.iter())
        .map(|(min, max)| max - min)
        .fold(0.0_f64, f64::max);
    let mut metrics = vec![
        metric("forward_displacement_m", "m", displacement_x_m, 0.5, 30.0),
        metric(
            "lateral_displacement_m",
            "m",
            lateral_displacement_m,
            0.05,
            20.0,
        ),
        metric(
            "maximum_absolute_steering_rad",
            "rad",
            maximum_abs_steering_rad,
            0.10,
            0.50,
        ),
        metric(
            "maximum_absolute_yaw_rate_rad_s",
            "rad/s",
            maximum_abs_yaw_rate_rad_s,
            0.05,
            3.0,
        ),
        metric(
            "maximum_front_rear_load_shift_n",
            "N",
            maximum_front_rear_load_shift_n,
            50.0,
            10_000.0,
        ),
        metric(
            "maximum_left_right_load_difference_n",
            "N",
            maximum_left_right_load_difference_n,
            20.0,
            10_000.0,
        ),
        metric(
            "maximum_split_mu_utilization_gap",
            "1",
            maximum_split_mu_utilization_gap,
            0.02,
            1.0,
        ),
        metric(
            "minimum_contact_fraction",
            "1",
            minimum_contact_fraction,
            0.70,
            1.0,
        ),
        metric(
            "maximum_suspension_travel_range_m",
            "m",
            maximum_suspension_travel_range_m,
            0.000_2,
            suspension.maximum_position_m - suspension.minimum_position_m
                + 2.0 * SUSPENSION_LIMIT_SOLVER_TOLERANCE_M,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = AckermannSuspensionTrace {
        kind: ACKERMANN_SUSPENSION_TRACE_KIND.to_string(),
        schema_version: ACKERMANN_SUSPENSION_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        wheel_plant_spec: plant,
        wheel_station_specs: stations.map(|station| station.spec),
        suspension_spec: suspension,
        steering_actuator_spec: steering_actuator,
        wheelbase_m: WHEELBASE_M,
        track_width_m: TRACK_WIDTH_M,
        fixed_delta_ticks: ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
        seed: WORLD_SEED,
        steps: TOTAL_STEPS,
        samples,
        passed: metrics.iter().all(|metric| metric.passed),
        metrics,
        content_digest: String::new(),
    };
    trace.content_digest = trace_digest(&trace)?;
    trace.validate()?;
    Ok(trace)
}

/// Builds a self-verifying SI-unit comparison from two complete traces.
pub fn compare_ackermann_suspension_traces(
    first: AckermannSuspensionTrace,
    second: AckermannSuspensionTrace,
) -> Result<AckermannSuspensionComparison> {
    first.validate()?;
    second.validate()?;
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = AckermannSuspensionComparison {
        kind: ACKERMANN_SUSPENSION_COMPARISON_KIND.to_string(),
        schema_version: ACKERMANN_SUSPENSION_SCHEMA_VERSION,
        first,
        second,
        passed: metrics.iter().all(|metric| metric.passed),
        metrics,
        content_digest: String::new(),
    };
    comparison.content_digest = comparison_digest(&comparison)?;
    comparison.validate()?;
    Ok(comparison)
}

pub(crate) fn spawn_stations(
    world: &mut World,
    chassis: Entity,
    suspension: SuspensionStrutSpec,
) -> Result<[StationEntities; 4]> {
    let names = ["front_left", "rear_left", "front_right", "rear_right"];
    let specs = wheel_station_specs();
    let suspension_actuation =
        evaluate_suspension_strut(suspension, INITIAL_SUSPENSION_POSITION_M, 0.0)?;
    Ok(std::array::from_fn(|index| {
        let spec = specs[index];
        let slider = spawn_named(world, format!("{}_slider", names[index]));
        let wheel_center_world_m =
            Vec3::new(spec.center_body_m.x, WHEEL_RADIUS_M, spec.center_body_m.z);
        world.entity_mut(slider).insert((
            RigidBody {
                mass_kg: suspension.unsprung_mass_kg * 0.4,
                ..RigidBody::default()
            },
            MultibodyLink,
            CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
            Transform3::from_translation_rotation(wheel_center_world_m, Quat::IDENTITY),
            PrismaticJointDesc {
                parent: chassis,
                axis: suspension.axis_body,
                anchor_parent_m: spec.center_body_m,
                anchor_child_m: Vec3::ZERO,
                lower_m: Some(suspension.minimum_position_m),
                upper_m: Some(suspension.maximum_position_m),
                relative_rotation: Quat::IDENTITY,
            },
            suspension,
            suspension_actuation,
        ));
        let wheel = spawn_named(world, names[index]);
        let front = index == 0 || index == 2;
        world.entity_mut(wheel).insert((
            RigidBody {
                mass_kg: suspension.unsprung_mass_kg * 0.6,
                ..RigidBody::default()
            },
            MultibodyLink,
            frictionless_sphere(WHEEL_RADIUS_M),
            CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
            Transform3::from_translation_rotation(wheel_center_world_m, Quat::IDENTITY),
            spec,
        ));
        let steering_limit_rad = if front { 0.5 } else { 1.0e-6 };
        world.entity_mut(wheel).insert((
            RevoluteJointDesc {
                parent: slider,
                axis: Vec3::Y,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::ZERO,
                lower_rad: Some(-steering_limit_rad),
                upper_rad: Some(steering_limit_rad),
                relative_rotation: Quat::IDENTITY,
            },
            JointActuation::RevolutePosition {
                target_position_rad: 0.0,
                stiffness_nm_per_rad: 1_500.0,
                damping_nm_s_per_rad: 120.0,
                max_effort_nm: 2_000.0,
            },
            JointMotorGainModel::ForceBased,
        ));
        StationEntities {
            slider,
            wheel,
            spec,
            front,
        }
    }))
}

pub(crate) fn wheel_station_specs() -> [WheelStationSpec; 4] {
    let centers = [
        Vec3::new(WHEELBASE_M * 0.5, -0.15, TRACK_WIDTH_M * 0.5),
        Vec3::new(-WHEELBASE_M * 0.5, -0.15, TRACK_WIDTH_M * 0.5),
        Vec3::new(WHEELBASE_M * 0.5, -0.15, -TRACK_WIDTH_M * 0.5),
        Vec3::new(-WHEELBASE_M * 0.5, -0.15, -TRACK_WIDTH_M * 0.5),
    ];
    std::array::from_fn(|index| WheelStationSpec {
        center_body_m: centers[index],
        maximum_steering_rad: if index == 0 || index == 2 { 0.5 } else { 0.0 },
        ..WheelStationSpec::default()
    })
}

pub(crate) fn suspension_spec() -> SuspensionStrutSpec {
    SuspensionStrutSpec {
        axis_body: Vec3::Y,
        // The unloaded free-length coordinate sits below static ride height, so
        // the grounded wheel carries spring preload instead of relying on a
        // numerically rigid zero-error servo to support chassis weight.
        equilibrium_position_m: -0.061,
        minimum_position_m: -0.06,
        maximum_position_m: 0.06,
        stiffness_n_per_m: 200_000.0,
        damping_n_s_per_m: 15_000.0,
        maximum_force_n: 50_000.0,
        unsprung_mass_kg: 24.0,
    }
}

pub(crate) fn steering_actuator_spec() -> SteeringActuatorSpec {
    SteeringActuatorSpec {
        time_constant_s: 0.08,
        maximum_rate_rad_s: 2.5,
        minimum_position_rad: -0.5,
        maximum_position_rad: 0.5,
        command_deadband_rad: 0.001,
        ..SteeringActuatorSpec::default()
    }
}

pub(crate) fn wheel_plant_spec() -> LongitudinalMobilityPlantSpec {
    let reference_load_n =
        (CHASSIS_MASS_KG + 4.0 * suspension_spec().unsprung_mass_kg) * 9.806_65 / 4.0;
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: CHASSIS_MASS_KG,
        driven_wheel_count: 1,
        normal_load_per_driven_wheel_n: reference_load_n,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.0,
        road_friction_scale: 1.0,
        motor: DcMotorSpec::default(),
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec {
            radius_m: WHEEL_RADIUS_M,
            width_m: 0.18,
            inertia_kg_m2: 1.2,
            rolling_resistance_coefficient: 0.012,
            ..WheelAssemblySpec::default()
        },
        tire: rne_robot::CombinedSlipTireSpec {
            reference_load_n,
            longitudinal_stiffness_n: 30_000.0,
            lateral_stiffness_n: 40_000.0,
            longitudinal_peak_friction: 0.9,
            lateral_peak_friction: 0.9,
            longitudinal_relaxation_length_m: 0.12,
            lateral_relaxation_length_m: 0.18,
            ..rne_robot::CombinedSlipTireSpec::default()
        },
    }
}

fn phase_for_step(step: u64) -> u8 {
    if step <= SETTLE_STEPS {
        0
    } else if step <= ACCEL_END_STEP {
        1
    } else if step <= TURN_END_STEP {
        2
    } else {
        3
    }
}

fn command_for_step(step: u64) -> (f64, [f64; 4]) {
    match phase_for_step(step) {
        0 => (0.0, [0.0; 4]),
        1 => {
            let ramp = ((step - SETTLE_STEPS) as f64 / 1_500.0).min(1.0);
            (0.0, [ramp * DRIVE_VOLTAGE_V; 4])
        }
        2 => {
            let ramp = ((step - ACCEL_END_STEP) as f64 / 1_000.0).min(1.0);
            (ramp * STEERING_COMMAND_RAD, [DRIVE_VOLTAGE_V; 4])
        }
        _ => {
            let ramp = ((step - TURN_END_STEP) as f64 / 800.0).min(1.0);
            (
                (1.0 - ramp) * STEERING_COMMAND_RAD,
                [-0.75 * DRIVE_VOLTAGE_V; 4],
            )
        }
    }
}

fn road_friction_scale(index: usize, step: u64) -> f64 {
    if step > TURN_END_STEP && index < 2 {
        0.45
    } else {
        1.0
    }
}

pub(crate) fn front_steering_targets(center_rad: f64) -> [f64; 2] {
    if center_rad.abs() < 1.0e-12 {
        return [0.0; 2];
    }
    let sign = center_rad.signum();
    let radius_m = WHEELBASE_M / center_rad.abs().tan();
    let inner = (WHEELBASE_M / (radius_m - TRACK_WIDTH_M * 0.5)).atan() * sign;
    let outer = (WHEELBASE_M / (radius_m + TRACK_WIDTH_M * 0.5)).atan() * sign;
    if center_rad > 0.0 {
        [outer, inner]
    } else {
        [inner, outer]
    }
}

#[allow(clippy::too_many_arguments)]
fn make_sample(
    step: u64,
    steering_command_rad: f64,
    steering_actuator_target_rad: f64,
    front_steering_rad: [f64; 2],
    command_voltage_v: [f64; 4],
    transform: Transform3,
    body: RigidBody,
    suspension_position_m: [f64; 4],
    suspension_velocity_m_s: [f64; 4],
    wheel_normal_load_n: [f64; 4],
    wheel_velocity_rad_s: [f64; 4],
    wheel_longitudinal_force_n: [f64; 4],
    wheel_lateral_force_n: [f64; 4],
    wheel_friction_utilization: [f64; 4],
) -> AckermannSuspensionSample {
    AckermannSuspensionSample {
        step,
        sim_time_ticks: SimTime::from_ticks(step * ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS).ticks(),
        phase: phase_for_step(step),
        steering_command_rad,
        steering_actuator_target_rad,
        front_steering_rad,
        command_voltage_v,
        privileged_position_world_m: transform.translation.to_array().map(f64::from),
        privileged_linear_velocity_world_m_s: body.linear_velocity_m_s.to_array().map(f64::from),
        privileged_angular_velocity_world_rad_s: body
            .angular_velocity_rad_s
            .to_array()
            .map(f64::from),
        suspension_position_m,
        suspension_velocity_m_s,
        wheel_normal_load_n,
        wheel_velocity_rad_s,
        wheel_longitudinal_force_n,
        wheel_lateral_force_n,
        wheel_friction_utilization,
    }
}

fn sample_zero(transform: Transform3) -> AckermannSuspensionSample {
    make_sample(
        0,
        0.0,
        0.0,
        [0.0; 2],
        [0.0; 4],
        transform,
        RigidBody::default(),
        [INITIAL_SUSPENSION_POSITION_M; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
    )
}

pub(crate) fn frictionless_cuboid(half_extents_m: Vec3) -> Collider {
    let mut collider = Collider::cuboid(half_extents_m);
    collider.material = PhysicsMaterial {
        friction: 0.0,
        restitution: 0.0,
    };
    collider
}

fn frictionless_sphere(radius_m: f64) -> Collider {
    let mut collider = Collider::sphere(radius_m);
    collider.material = PhysicsMaterial {
        friction: 0.0,
        restitution: 0.0,
    };
    collider
}

fn metric(id: &str, unit: &str, value: f64, minimum: f64, maximum: f64) -> MobilityBenchmarkMetric {
    MobilityBenchmarkMetric {
        id: id.to_string(),
        unit: unit.to_string(),
        value,
        minimum,
        maximum,
        passed: value.is_finite() && (minimum..=maximum).contains(&value),
    }
}

fn validate_metrics(metrics: &[MobilityBenchmarkMetric], passed: bool) -> Result<()> {
    ensure!(!metrics.is_empty(), "missing metrics");
    ensure!(
        metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
        "metrics are not canonical"
    );
    for metric in metrics {
        ensure!(
            metric.value.is_finite()
                && metric.minimum.is_finite()
                && metric.maximum.is_finite()
                && metric.minimum <= metric.maximum,
            "invalid metric {}",
            metric.id
        );
        ensure!(
            metric.passed == (metric.minimum..=metric.maximum).contains(&metric.value),
            "metric verdict drift"
        );
    }
    ensure!(
        passed == metrics.iter().all(|metric| metric.passed),
        "verdict drift"
    );
    Ok(())
}

fn comparison_metrics(
    first: &AckermannSuspensionTrace,
    second: &AckermannSuspensionTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    let mut metrics = vec![
        gap_metric(
            "forward_displacement_gap_m",
            "m",
            first,
            second,
            "forward_displacement_m",
            0.05,
        )?,
        gap_metric(
            "lateral_displacement_gap_m",
            "m",
            first,
            second,
            "lateral_displacement_m",
            0.03,
        )?,
        gap_metric(
            "yaw_rate_gap_rad_s",
            "rad/s",
            first,
            second,
            "maximum_absolute_yaw_rate_rad_s",
            0.02,
        )?,
        gap_metric(
            "front_rear_load_shift_gap_n",
            "N",
            first,
            second,
            "maximum_front_rear_load_shift_n",
            100.0,
        )?,
        gap_metric(
            "left_right_load_difference_gap_n",
            "N",
            first,
            second,
            "maximum_left_right_load_difference_n",
            75.0,
        )?,
        gap_metric(
            "suspension_travel_range_gap_m",
            "m",
            first,
            second,
            "maximum_suspension_travel_range_m",
            0.001,
        )?,
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    Ok(metrics)
}

fn gap_metric(
    id: &str,
    unit: &str,
    first: &AckermannSuspensionTrace,
    second: &AckermannSuspensionTrace,
    source_id: &str,
    maximum: f64,
) -> Result<MobilityBenchmarkMetric> {
    let gap = (metric_value(first, source_id)? - metric_value(second, source_id)?).abs();
    Ok(metric(id, unit, gap, 0.0, maximum))
}

fn metric_value(trace: &AckermannSuspensionTrace, id: &str) -> Result<f64> {
    trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .map(|metric| metric.value)
        .with_context(|| format!("missing metric {id}"))
}

fn trace_digest(trace: &AckermannSuspensionTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn comparison_digest(comparison: &AckermannSuspensionComparison) -> Result<String> {
    let mut canonical = comparison.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    format!("fnv1a64:{digest:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn ackermann_geometry_has_distinct_inner_and_outer_angles() {
        let positive = front_steering_targets(STEERING_COMMAND_RAD);
        assert!(positive[1] > positive[0]);
        let negative = front_steering_targets(-STEERING_COMMAND_RAD);
        assert!(negative[0] < negative[1]);
        assert!((positive[0] + negative[1]).abs() < 1.0e-12);
        assert!((positive[1] + negative[0]).abs() < 1.0e-12);
    }

    #[test]
    fn rapier_ackermann_suspension_trace_is_passing_and_deterministic() {
        let first = run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())
            .unwrap();
        assert!(first.passed, "{:#?}", first.metrics);
        first.validate().unwrap();
        let second =
            run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())
                .unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn trace_tampering_is_detected() {
        let mut trace =
            run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())
                .unwrap();
        trace.samples[1].suspension_position_m[0] += 0.001;
        assert!(trace.validate().is_err());
    }

    #[test]
    fn steering_actuator_target_must_match_deterministic_replay() {
        let mut trace =
            run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())
                .unwrap();
        let turn_sample = trace
            .samples
            .iter_mut()
            .find(|sample| sample.phase == 2)
            .unwrap();
        turn_sample.steering_actuator_target_rad += 0.001;
        trace.content_digest = trace_digest(&trace).unwrap();

        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_ackermann_suspension_traces_pass() {
        use rne_physics_mujoco::MuJoCoBackend;

        let rapier =
            run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())
                .unwrap();
        let mujoco = run_ackermann_suspension_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(
                ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
            ))
            .unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_ackermann_suspension_traces(rapier, mujoco).unwrap();
        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();
    }
}
