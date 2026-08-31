//! Same-TaskSpec longitudinal contact/wrench execution across physics backends.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    ActionSpec, BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction,
    BehaviorReplayArtifact, BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation,
    ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds, TensorDType,
    TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, World};
use rne_math::{y_up_euler_rad, Quat, Vec3};
use rne_physics::{
    require_capabilities, Collider, PhysicsBackend, PhysicsBackendManifest, PhysicsCapability,
    PhysicsMaterial, PhysicsWorldDesc, RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, CombinedSlipTireSpec,
    DcMotorSpec, LongitudinalDrivePathInput, LongitudinalDrivePathState,
    LongitudinalMobilityPlantSpec, TransmissionSpec, WheelAssemblySpec,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

use crate::MobilityBenchmarkMetric;

/// Artifact discriminator for one backend mobility trace.
pub const BACKEND_MOBILITY_TRACE_KIND: &str = "rne_mobility_backend_trace";
/// Current backend trace schema version.
pub const BACKEND_MOBILITY_TRACE_SCHEMA_VERSION: u32 = 1;
/// Artifact discriminator for a unit-aware cross-backend comparison.
pub const BACKEND_MOBILITY_COMPARISON_KIND: &str = "rne_mobility_backend_comparison";
/// Current cross-backend comparison schema version.
pub const BACKEND_MOBILITY_COMPARISON_SCHEMA_VERSION: u32 = 1;
/// Stable identity shared by every backend run.
pub const BACKEND_MOBILITY_TASK_ID: &str = "mobility_longitudinal_contact_wrench_v1";
/// Fixed backend and plant step in simulation nanosecond ticks.
pub const BACKEND_MOBILITY_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SETTLE_STEPS: u64 = 300;
const DRIVE_STEPS: u64 = 2_000;
const TOTAL_STEPS: u64 = SETTLE_STEPS + DRIVE_STEPS;
const COMMAND_VOLTAGE_V: f64 = 24.0;
const WORLD_SEED: u64 = 0;
const TRACE_SAMPLE_STRIDE_STEPS: u64 = 100;

/// One sampled row; fields prefixed `privileged_` are metrics-only truth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BackendMobilitySample {
    /// One-based completed physics step.
    pub step: u64,
    /// Completed simulation time in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Task-provided actor observation, zero while settling and one while driving.
    pub command_phase: f64,
    /// Bounded TaskSpec action in volts.
    pub command_voltage_v: f64,
    /// Privileged chassis position in world coordinates, in meters.
    pub privileged_position_world_m: [f64; 3],
    /// Privileged chassis velocity in world coordinates, in meters per second.
    pub privileged_velocity_world_m_s: [f64; 3],
    /// Privileged chassis orientation quaternion in `[x, y, z, w]` order.
    pub privileged_rotation_xyzw: [f64; 4],
    /// Privileged chassis angular velocity in world coordinates, in radians per second.
    pub privileged_angular_velocity_world_rad_s: [f64; 3],
    /// Completed representative wheel velocity in radians per second.
    pub wheel_velocity_rad_s: f64,
    /// Completed equivalent motor current in amperes.
    pub motor_current_a: f64,
    /// Completed backend normal load in newtons, zero while unsupported.
    pub contact_normal_load_n: f64,
    /// Tire force scheduled for the following backend step, in newtons.
    pub tire_longitudinal_force_n: f64,
    /// Combined tire friction utilization, bounded by one.
    pub friction_utilization: f64,
}

impl BackendMobilitySample {
    fn is_finite(&self) -> bool {
        self.command_phase.is_finite()
            && self.command_voltage_v.is_finite()
            && self
                .privileged_position_world_m
                .iter()
                .chain(self.privileged_velocity_world_m_s.iter())
                .chain(self.privileged_rotation_xyzw.iter())
                .chain(self.privileged_angular_velocity_world_rad_s.iter())
                .all(|value| value.is_finite())
            && self.wheel_velocity_rad_s.is_finite()
            && self.motor_current_a.is_finite()
            && self.contact_normal_load_n.is_finite()
            && self.tire_longitudinal_force_n.is_finite()
            && self.friction_utilization.is_finite()
    }
}

/// Deterministic TaskSpec-bound trace from one physics backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BackendMobilityTrace {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Trace schema version.
    pub schema_version: u32,
    /// Exact backend identity and capability declaration.
    pub backend: PhysicsBackendManifest,
    /// Exact portable task contract executed by this run.
    pub task_spec: TaskSpec,
    /// Fixed step in simulation nanosecond ticks.
    pub fixed_delta_ticks: u64,
    /// Explicit deterministic world seed.
    pub seed: u64,
    /// Number of completed physics steps.
    pub steps: u64,
    /// Ordered downsampled time series including the final step.
    pub samples: Vec<BackendMobilitySample>,
    /// Ordered SI-unit acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every acceptance metric passed.
    pub passed: bool,
    /// FNV-1a digest of the same trace with this field empty.
    pub content_digest: String,
}

impl BackendMobilityTrace {
    /// Recomputes schema, task, time-series, verdict, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == BACKEND_MOBILITY_TRACE_KIND, "kind mismatch");
        ensure!(
            self.schema_version == BACKEND_MOBILITY_TRACE_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.backend.validate().context("backend manifest")?;
        self.task_spec.validate().context("TaskSpec")?;
        ensure!(
            self.task_spec == backend_mobility_task_spec(),
            "exact TaskSpec mismatch"
        );
        ensure!(
            self.fixed_delta_ticks == BACKEND_MOBILITY_FIXED_DELTA_TICKS,
            "fixed step mismatch"
        );
        ensure!(self.steps == TOTAL_STEPS, "step count mismatch");
        ensure!(self.seed == WORLD_SEED, "world seed mismatch");
        ensure!(!self.samples.is_empty(), "trace omitted samples");
        ensure!(
            self.samples
                .windows(2)
                .all(|pair| pair[0].step < pair[1].step),
            "samples are not strictly ordered"
        );
        ensure!(
            self.samples
                .last()
                .is_some_and(|sample| sample.step == self.steps),
            "trace omitted final step"
        );
        for sample in &self.samples {
            ensure!(sample.is_finite(), "sample {} is non-finite", sample.step);
            ensure!(
                sample.sim_time_ticks == sample.step * self.fixed_delta_ticks,
                "sample {} timestamp mismatch",
                sample.step
            );
            let expected_phase = if sample.step > SETTLE_STEPS { 1.0 } else { 0.0 };
            ensure!(
                sample.command_phase == expected_phase
                    && sample.command_voltage_v == expected_phase * COMMAND_VOLTAGE_V,
                "sample {} command schedule mismatch",
                sample.step
            );
            ensure!(
                (0.0..=1.0).contains(&sample.friction_utilization),
                "sample {} utilization escaped bounds",
                sample.step
            );
        }
        ensure!(!self.metrics.is_empty(), "trace omitted metrics");
        ensure!(
            self.metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
            "metrics are not strictly sorted"
        );
        for metric in &self.metrics {
            ensure!(
                metric.value.is_finite(),
                "metric {} is non-finite",
                metric.id
            );
            ensure!(
                metric.minimum.is_finite()
                    && metric.maximum.is_finite()
                    && metric.minimum <= metric.maximum,
                "metric {} interval is invalid",
                metric.id
            );
            ensure!(
                metric.passed == (metric.value >= metric.minimum && metric.value <= metric.maximum),
                "metric {} verdict mismatch",
                metric.id
            );
        }
        ensure!(
            self.passed == self.metrics.iter().all(|metric| metric.passed),
            "trace verdict mismatch"
        );
        ensure!(
            self.content_digest == trace_digest(self)?,
            "trace digest mismatch"
        );
        Ok(())
    }
}

/// Two self-verifying traces plus explicit SI-unit cross-backend tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BackendMobilityComparison {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Comparison schema version.
    pub schema_version: u32,
    /// First complete backend trace.
    pub first: BackendMobilityTrace,
    /// Second complete backend trace.
    pub second: BackendMobilityTrace,
    /// Ordered absolute backend gaps with unit-bearing acceptance intervals.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every cross-backend tolerance passed.
    pub passed: bool,
    /// FNV-1a digest of the same comparison with this field empty.
    pub content_digest: String,
}

impl BackendMobilityComparison {
    /// Recomputes trace integrity, shared contracts, tolerances, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == BACKEND_MOBILITY_COMPARISON_KIND,
            "comparison kind mismatch"
        );
        ensure!(
            self.schema_version == BACKEND_MOBILITY_COMPARISON_SCHEMA_VERSION,
            "comparison schema mismatch"
        );
        self.first.validate().context("first backend trace")?;
        self.second.validate().context("second backend trace")?;
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "comparison requires distinct backend identities"
        );
        ensure!(
            self.first.task_spec == self.second.task_spec
                && self.first.fixed_delta_ticks == self.second.fixed_delta_ticks
                && self.first.seed == self.second.seed,
            "backend execution contract mismatch"
        );
        let expected = comparison_metrics(&self.first, &self.second)?;
        ensure!(self.metrics == expected, "comparison metric drift");
        ensure!(
            self.passed == self.metrics.iter().all(|metric| metric.passed),
            "comparison verdict mismatch"
        );
        ensure!(
            self.content_digest == comparison_digest(self)?,
            "comparison digest mismatch"
        );
        Ok(())
    }
}

/// Compares two complete runs without hiding backend-specific divergence.
pub fn compare_backend_mobility_traces(
    first: BackendMobilityTrace,
    second: BackendMobilityTrace,
) -> Result<BackendMobilityComparison> {
    first.validate().context("first backend trace")?;
    second.validate().context("second backend trace")?;
    ensure!(
        first.backend.backend_id != second.backend.backend_id,
        "comparison requires distinct backend identities"
    );
    ensure!(
        first.task_spec == second.task_spec
            && first.fixed_delta_ticks == second.fixed_delta_ticks
            && first.seed == second.seed,
        "backend execution contract mismatch"
    );
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = BackendMobilityComparison {
        kind: BACKEND_MOBILITY_COMPARISON_KIND.to_string(),
        schema_version: BACKEND_MOBILITY_COMPARISON_SCHEMA_VERSION,
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

/// Builds a compact failure replay for a deliberately stricter position bound.
///
/// Production acceptance remains in [`compare_backend_mobility_traces`]. This
/// diagnostic exists to prove that a real backend divergence can be retained
/// through RNE's standard Behavior replay and Failure Capsule pipeline.
pub fn backend_mobility_divergence_replay(
    first: &BackendMobilityTrace,
    second: &BackendMobilityTrace,
    diagnostic_position_tolerance_m: f64,
) -> Result<BehaviorReplayArtifact> {
    first.validate().context("first backend trace")?;
    second.validate().context("second backend trace")?;
    ensure!(
        first.task_spec == second.task_spec
            && first.fixed_delta_ticks == second.fixed_delta_ticks
            && first.seed == second.seed,
        "backend execution contract mismatch"
    );
    ensure!(
        diagnostic_position_tolerance_m.is_finite() && diagnostic_position_tolerance_m > 0.0,
        "diagnostic position tolerance must be finite and positive"
    );
    ensure!(
        first.samples.len() == second.samples.len(),
        "backend sample counts differ"
    );

    let descriptor = BehaviorContractDescriptor {
        name: "mobility_cross_backend_forward_position_bound".to_string(),
        kind: BehaviorContractKind::Always,
        entities: vec![
            first.backend.backend_id.clone(),
            second.backend.backend_id.clone(),
        ],
    };
    let mut frames = Vec::new();
    let mut violation = None;
    for (index, (first_sample, second_sample)) in
        first.samples.iter().zip(&second.samples).enumerate()
    {
        ensure!(
            first_sample.step == second_sample.step,
            "backend sample steps differ"
        );
        let position_gap_m = (first_sample.privileged_position_world_m[0]
            - second_sample.privileged_position_world_m[0])
            .abs();
        let observation = serde_json::json!({
            "task_id": first.task_spec.task_id,
            "source_step": first_sample.step,
            "first_backend": first.backend.backend_id,
            "second_backend": second.backend.backend_id,
            "first_forward_position_m": first_sample.privileged_position_world_m[0],
            "second_forward_position_m": second_sample.privileged_position_world_m[0],
            "absolute_forward_position_gap_m": position_gap_m,
            "diagnostic_position_tolerance_m": diagnostic_position_tolerance_m,
        });
        let state_digest = fnv1a64(&serde_json::to_vec(&observation)?);
        let frame = BehaviorReplayFrame {
            step: index as u64,
            sim_time_ticks: index as u64 * first.fixed_delta_ticks * TRACE_SAMPLE_STRIDE_STEPS,
            action: if index == 0 {
                BehaviorReplayAction::InitialObservation
            } else {
                BehaviorReplayAction::Advance
            },
            observation,
            state_digest,
        };
        if position_gap_m > diagnostic_position_tolerance_m {
            violation = Some(BehaviorViolation {
                step: frame.step,
                sim_time_ticks: frame.sim_time_ticks,
                state_digest,
                entities: descriptor.entities.clone(),
                message: format!(
                    "cross-backend forward-position gap {position_gap_m:.12} m exceeded injected diagnostic bound {diagnostic_position_tolerance_m:.12} m"
                ),
            });
            frames.push(frame);
            break;
        }
        frames.push(frame);
    }
    let violation = violation.context("diagnostic tolerance produced no divergence")?;
    let scenario_digest = fnv1a64(&serde_json::to_vec(&serde_json::json!({
        "task_spec": first.task_spec,
        "diagnostic_position_tolerance_m": diagnostic_position_tolerance_m,
        "first_backend": first.backend,
        "second_backend": second.backend,
    }))?);
    Ok(BehaviorReplayArtifact::new(
        "mobility_rapier_vs_mujoco_longitudinal",
        scenario_digest,
        first.seed,
        first.fixed_delta_ticks * TRACE_SAMPLE_STRIDE_STEPS,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor,
            violation,
        },
    )?)
}

/// Returns the exact open-loop actor/action contract shared across backends.
///
/// `command_phase` is task data rather than simulator state. All chassis fields
/// in [`BackendMobilitySample`] remain explicitly privileged metrics and are not
/// part of this actor-visible observation.
pub fn backend_mobility_task_spec() -> TaskSpec {
    TaskSpec::new(
        BACKEND_MOBILITY_TASK_ID,
        BACKEND_MOBILITY_FIXED_DELTA_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![TensorSpec::new(
            "command_phase",
            TensorDType::F64,
            vec![],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(0.0, 1.0))]),
        ActionSpec::new(vec![TensorSpec::new(
            "motor_terminal_voltage_v",
            TensorDType::F64,
            vec![1],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(
            -COMMAND_VOLTAGE_V,
            COMMAND_VOLTAGE_V,
        ))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_forward_progress_m", 1.0, "m"),
            RewardTermSpec::new("task_step", -0.001, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "truth_out_of_bounds",
                TerminationKind::Failure,
            )],
            Some(TOTAL_STEPS),
        ),
        ResetSpec::splitmix64(false),
    )
}

/// Runs the shared contact-to-tire-to-wrench loop on one backend.
pub fn run_backend_mobility_trace<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<BackendMobilityTrace> {
    manifest.validate().context("backend manifest")?;
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::ContactForce,
            PhysicsCapability::ExternalBodyWrench,
            PhysicsCapability::ContactPointKinematics,
        ],
    )?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "backend manifest capability drift"
    );
    let task_spec = backend_mobility_task_spec();
    task_spec.validate().context("TaskSpec")?;
    let fixed_delta = SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 16,
    })?;
    let mut world = World::new();
    let ground = spawn_named(&mut world, "mobility_benchmark_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        frictionless_collider(Vec3::new(20.0, 0.5, 5.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let vehicle = spawn_named(&mut world, "mobility_benchmark_vehicle");
    world.entity_mut(vehicle).insert((
        RigidBody {
            mass_kg: 100.0,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: 5.083_333_333_333_333,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 11.333_333_333_333_334,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 10.416_666_666_666_666,
        },
        frictionless_collider(Vec3::new(0.5, 0.25, 0.3)),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.251, 0.0), Quat::IDENTITY),
    ));
    backend.sync_from_ecs(&mut world, physics_world)?;

    let plant = backend_plant_spec();
    let mut drive_state = LongitudinalDrivePathState::default();
    let mut pending_wrench = None;
    let mut samples = Vec::new();
    let mut contact_drive_steps = 0_u64;
    let mut maximum_current_a = 0.0_f64;
    let mut maximum_utilization = 0.0_f64;
    let mut maximum_vertical_displacement_m = 0.0_f64;
    let mut maximum_tilt_rad = 0.0_f64;
    let initial_position = *world
        .get::<Transform3>(vehicle)
        .context("vehicle transform before run")?;
    let initial_body = *world
        .get::<RigidBody>(vehicle)
        .context("vehicle body before run")?;
    samples.push(BackendMobilitySample {
        step: 0,
        sim_time_ticks: 0,
        command_phase: 0.0,
        command_voltage_v: 0.0,
        privileged_position_world_m: initial_position.translation.to_array().map(f64::from),
        privileged_velocity_world_m_s: initial_body.linear_velocity_m_s.to_array().map(f64::from),
        privileged_rotation_xyzw: initial_position.rotation.to_array().map(f64::from),
        privileged_angular_velocity_world_rad_s: initial_body
            .angular_velocity_rad_s
            .to_array()
            .map(f64::from),
        wheel_velocity_rad_s: 0.0,
        motor_current_a: 0.0,
        contact_normal_load_n: 0.0,
        tire_longitudinal_force_n: 0.0,
        friction_utilization: 0.0,
    });

    for zero_based_step in 0..TOTAL_STEPS {
        if let Some(wrench) = pending_wrench.take() {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;

        let command_phase = if zero_based_step >= SETTLE_STEPS {
            1.0
        } else {
            0.0
        };
        let command_voltage_v = command_phase * COMMAND_VOLTAGE_V;
        let carrier_patch = aggregate_wheel_contact_patch(
            vehicle,
            backend.contact_points(physics_world)?,
            Vec3::X,
            Vec3::Z,
        )?;
        if zero_based_step >= SETTLE_STEPS && carrier_patch.is_some() {
            contact_drive_steps += 1;
        }
        let normal_load_n = carrier_patch.map_or(0.0, |patch| patch.normal_load_n);
        let drive = evaluate_longitudinal_drive_path(
            plant,
            drive_state,
            LongitudinalDrivePathInput {
                carrier_patch,
                forward_world: Vec3::X,
                lateral_world: Vec3::Z,
                command_voltage_v,
            },
            dt_s,
        )?;
        drive_state = drive.state;
        pending_wrench = drive.tire_wrench;
        let tire_force_n = drive.tire.longitudinal_force_n;
        maximum_current_a = maximum_current_a.max(drive.motor.state.current_a.abs());
        maximum_utilization = maximum_utilization.max(drive.tire.friction_utilization);

        let transform = *world
            .get::<Transform3>(vehicle)
            .context("vehicle transform after step")?;
        let (_, pitch_rad, roll_rad) = y_up_euler_rad(transform.rotation);
        maximum_vertical_displacement_m = maximum_vertical_displacement_m
            .max((transform.translation.y - initial_position.translation.y).abs());
        maximum_tilt_rad = maximum_tilt_rad.max(pitch_rad.hypot(roll_rad));

        let step = zero_based_step + 1;
        if step % TRACE_SAMPLE_STRIDE_STEPS == 0 || step == TOTAL_STEPS {
            let body = *world
                .get::<RigidBody>(vehicle)
                .context("vehicle body after step")?;
            samples.push(BackendMobilitySample {
                step,
                sim_time_ticks: SimTime::from_ticks(step * BACKEND_MOBILITY_FIXED_DELTA_TICKS)
                    .ticks(),
                command_phase,
                command_voltage_v,
                privileged_position_world_m: transform.translation.to_array().map(f64::from),
                privileged_velocity_world_m_s: body.linear_velocity_m_s.to_array().map(f64::from),
                privileged_rotation_xyzw: transform.rotation.to_array().map(f64::from),
                privileged_angular_velocity_world_rad_s: body
                    .angular_velocity_rad_s
                    .to_array()
                    .map(f64::from),
                wheel_velocity_rad_s: drive_state.wheel_velocity_rad_s,
                motor_current_a: drive.motor.state.current_a,
                contact_normal_load_n: normal_load_n,
                tire_longitudinal_force_n: tire_force_n,
                friction_utilization: drive.tire.friction_utilization,
            });
        }
    }

    let final_transform = *world
        .get::<Transform3>(vehicle)
        .context("vehicle final transform")?;
    let final_body = *world
        .get::<RigidBody>(vehicle)
        .context("vehicle final body")?;
    let mut metrics = vec![
        metric(
            "contact_drive_fraction",
            "1",
            contact_drive_steps as f64 / DRIVE_STEPS as f64,
            0.95,
            1.0,
        ),
        metric(
            "final_forward_distance_m",
            "m",
            final_transform.translation.x - initial_position.translation.x,
            0.2,
            20.0,
        ),
        metric(
            "final_forward_velocity_m_s",
            "m/s",
            final_body.linear_velocity_m_s.x,
            0.1,
            20.0,
        ),
        metric(
            "final_lateral_drift_m",
            "m",
            final_transform.translation.z.abs(),
            0.0,
            0.05,
        ),
        metric(
            "maximum_motor_current_a",
            "A",
            maximum_current_a,
            19.9,
            20.0,
        ),
        metric("maximum_tilt_rad", "rad", maximum_tilt_rad, 0.0, 0.02),
        metric(
            "maximum_tire_utilization",
            "1",
            maximum_utilization,
            0.05,
            1.0,
        ),
        metric(
            "maximum_vertical_displacement_m",
            "m",
            maximum_vertical_displacement_m,
            0.0,
            0.01,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = BackendMobilityTrace {
        kind: BACKEND_MOBILITY_TRACE_KIND.to_string(),
        schema_version: BACKEND_MOBILITY_TRACE_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        fixed_delta_ticks: BACKEND_MOBILITY_FIXED_DELTA_TICKS,
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

fn backend_plant_spec() -> LongitudinalMobilityPlantSpec {
    let static_load_n = 100.0 * 9.806_65;
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: 100.0,
        driven_wheel_count: 1,
        normal_load_per_driven_wheel_n: static_load_n,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.0,
        road_friction_scale: 1.0,
        motor: DcMotorSpec::default(),
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec::default(),
        tire: CombinedSlipTireSpec {
            reference_load_n: static_load_n,
            ..CombinedSlipTireSpec::default()
        },
    }
}

fn frictionless_collider(half_extents_m: Vec3) -> Collider {
    let mut collider = Collider::cuboid(half_extents_m);
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
        passed: value >= minimum && value <= maximum,
    }
}

fn trace_digest(trace: &BackendMobilityTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    let bytes = serde_json::to_vec(&canonical)?;
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

fn comparison_metrics(
    first: &BackendMobilityTrace,
    second: &BackendMobilityTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    let specifications = [
        ("contact_drive_fraction", "1", 0.01),
        ("final_forward_distance_m", "m", 0.05),
        ("final_forward_velocity_m_s", "m/s", 0.05),
        ("final_lateral_drift_m", "m", 0.01),
        ("maximum_motor_current_a", "A", 0.05),
        ("maximum_tilt_rad", "rad", 0.005),
        ("maximum_tire_utilization", "1", 0.1),
        ("maximum_vertical_displacement_m", "m", 0.005),
    ];
    specifications
        .into_iter()
        .map(|(id, unit, tolerance)| {
            let first_value = metric_value(first, id, unit)?;
            let second_value = metric_value(second, id, unit)?;
            Ok(metric(
                &format!("absolute_{id}_gap"),
                unit,
                (first_value - second_value).abs(),
                0.0,
                tolerance,
            ))
        })
        .collect()
}

fn metric_value(trace: &BackendMobilityTrace, id: &str, unit: &str) -> Result<f64> {
    let metric = trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .with_context(|| format!("backend trace omitted metric {id}"))?;
    ensure!(metric.unit == unit, "metric {id} unit mismatch");
    Ok(metric.value)
}

fn comparison_digest(comparison: &BackendMobilityComparison) -> Result<String> {
    let mut canonical = comparison.clone();
    canonical.content_digest.clear();
    let bytes = serde_json::to_vec(&canonical)?;
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    digest
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn rapier_runs_the_exact_task_and_emits_a_self_verifying_trace() {
        let first = run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest())
            .expect("Rapier mobility trace");
        let second = run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest())
            .expect("Rapier mobility replay");

        assert!(first.passed, "{:#?}", first.metrics);
        assert_eq!(first, second);
        first.validate().unwrap();
    }

    #[test]
    fn trace_tampering_is_detected() {
        let mut trace =
            run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace
            .samples
            .last_mut()
            .unwrap()
            .privileged_position_world_m[0] += 0.5;
        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn mujoco_runs_the_same_task_contract() {
        use rne_physics_mujoco::MuJoCoBackend;

        let backend =
            MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))
                .expect("MuJoCo runtime");
        let trace = run_backend_mobility_trace(backend, MuJoCoBackend::manifest())
            .expect("MuJoCo mobility trace");
        assert!(trace.passed, "{:#?}", trace.metrics);
        assert_eq!(trace.task_spec, backend_mobility_task_spec());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_stay_within_unit_aware_tolerances() {
        use rne_physics_mujoco::MuJoCoBackend;

        let rapier =
            run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_backend_mobility_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))
                .expect("MuJoCo runtime"),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_backend_mobility_traces(rapier, mujoco).unwrap();

        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();

        let replay =
            backend_mobility_divergence_replay(&comparison.first, &comparison.second, 0.001)
                .expect("diagnostic replay");
        replay.validate().unwrap();
        assert!(replay.failure.violation.message.contains("0.001000000000"));
    }
}
