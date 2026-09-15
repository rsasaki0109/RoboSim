//! Differential-drive contact dynamics with an explicit passive trailing caster.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, Collider, CollisionGroups, ExternalBodyWrench, FixedJointDesc,
    JointPassiveDynamics, JointState, MultibodyLink, PhysicsBackend, PhysicsBackendManifest,
    PhysicsCapability, PhysicsMaterial, PhysicsWorldDesc, RevoluteJointDesc, RigidBody,
    RigidBodyInertia, RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, combined_slip_tire_wrench, evaluate_combined_slip_tire,
    evaluate_longitudinal_drive_path, resolve_wheel_station_frame, CombinedSlipTireInput,
    CombinedSlipTireSpec, CombinedSlipTireState, DcMotorSpec, LongitudinalDrivePathInput,
    LongitudinalDrivePathState, LongitudinalMobilityPlantSpec, PassiveCasterSpec, TransmissionSpec,
    WheelAssemblySpec, WheelStationSpec,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

use crate::MobilityBenchmarkMetric;

/// Artifact discriminator for one differential-caster trace.
pub const DIFF_CASTER_TRACE_KIND: &str = "rne_mobility_diff_caster_trace";
/// Artifact discriminator for a cross-backend differential-caster comparison.
pub const DIFF_CASTER_COMPARISON_KIND: &str = "rne_mobility_diff_caster_comparison";
/// Current artifact schema version.
pub const DIFF_CASTER_SCHEMA_VERSION: u32 = 1;
/// Stable task identity shared by all backends.
pub const DIFF_CASTER_TASK_ID: &str = "mobility_diff_drive_trailing_caster_v1";
/// Fixed physics and force-element step in simulation nanosecond ticks.
pub const DIFF_CASTER_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SETTLE_STEPS: u64 = 1_000;
const ARC_END_STEP: u64 = 2_200;
const ACCEL_END_STEP: u64 = 3_400;
const TOTAL_STEPS: u64 = 4_200;
const TRACE_STRIDE_STEPS: u64 = 100;
const COMMAND_VOLTAGE_V: f64 = 8.0;
const WORLD_SEED: u64 = 0;
const DRIVE_WHEEL_RADIUS_M: f64 = 0.12;
const CONTACT_LOAD_FILTER_TIME_CONSTANT_S: f64 = 0.02;
const SELF_COLLISION_GROUP: u32 = 1;

/// One unit-bearing sample; chassis pose and velocity are privileged scoring truth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DifferentialCasterSample {
    /// One-based completed step, or zero for the initial state.
    pub step: u64,
    /// Completed simulation time in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Frozen maneuver phase: settle, differential arc, straighten, or reverse.
    pub command_phase: u8,
    /// Left/right motor terminal-voltage action in volts.
    pub command_voltage_v: [f64; 2],
    /// Privileged chassis position in world coordinates, in meters.
    pub privileged_position_world_m: [f64; 3],
    /// Privileged unwrapped yaw, in radians.
    pub privileged_integrated_yaw_rad: f64,
    /// Privileged chassis forward velocity, in meters per second.
    pub privileged_forward_velocity_m_s: f64,
    /// Completed left/right driven-wheel normal loads, in newtons.
    pub drive_normal_load_n: [f64; 2],
    /// Completed caster normal load, in newtons.
    pub caster_normal_load_n: f64,
    /// Whether the caster had solved load-bearing contact on this step.
    pub caster_in_contact: bool,
    /// Completed caster swivel orientation wrapped to `[-pi, pi]`, in radians.
    pub caster_swivel_rad: f64,
    /// Completed unwrapped caster swivel coordinate, in radians.
    pub caster_swivel_unwrapped_rad: f64,
    /// Completed caster swivel rate, in radians per second.
    pub caster_swivel_velocity_rad_s: f64,
    /// Completed caster rolling coordinate rate, in radians per second.
    pub caster_roll_velocity_rad_s: f64,
    /// Caster lateral force scheduled for the next physics step, in newtons.
    pub caster_lateral_force_n: f64,
    /// Caster combined-friction utilization.
    pub caster_friction_utilization: f64,
}

impl DifferentialCasterSample {
    fn is_finite(&self) -> bool {
        self.command_voltage_v
            .iter()
            .chain(self.privileged_position_world_m.iter())
            .chain(self.drive_normal_load_n.iter())
            .all(|value| value.is_finite())
            && self.privileged_integrated_yaw_rad.is_finite()
            && self.privileged_forward_velocity_m_s.is_finite()
            && self.caster_normal_load_n.is_finite()
            && self.caster_swivel_rad.is_finite()
            && self.caster_swivel_unwrapped_rad.is_finite()
            && self.caster_swivel_velocity_rad_s.is_finite()
            && self.caster_roll_velocity_rad_s.is_finite()
            && self.caster_lateral_force_n.is_finite()
            && self.caster_friction_utilization.is_finite()
    }
}

/// Complete deterministic differential-drive/caster execution evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DifferentialCasterTrace {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// Exact backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact portable task executed by this run.
    pub task_spec: TaskSpec,
    /// Motor/transmission/wheel/tire contract for each driven wheel.
    pub drive_plant_spec: LongitudinalMobilityPlantSpec,
    /// Ordered left/right driven-wheel geometry.
    pub drive_station_specs: [WheelStationSpec; 2],
    /// Explicit swivel, trail, rolling inertia, damping, and tire contract.
    pub caster_spec: PassiveCasterSpec,
    /// Fixed simulation step in nanosecond ticks.
    pub fixed_delta_ticks: u64,
    /// Explicit world seed.
    pub seed: u64,
    /// Completed physics steps.
    pub steps: u64,
    /// Ordered downsampled physical evidence.
    pub samples: Vec<DifferentialCasterSample>,
    /// Ordered unit-bearing acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every acceptance metric passed.
    pub passed: bool,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl DifferentialCasterTrace {
    /// Recomputes frozen contracts, sample invariants, metrics, verdict, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == DIFF_CASTER_TRACE_KIND, "kind mismatch");
        ensure!(
            self.schema_version == DIFF_CASTER_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.backend.validate()?;
        self.task_spec.validate()?;
        ensure!(
            self.task_spec == differential_caster_task_spec(),
            "TaskSpec drift"
        );
        ensure!(
            self.drive_plant_spec == drive_plant_spec(),
            "drive plant drift"
        );
        ensure!(
            self.drive_station_specs == drive_station_specs(),
            "station drift"
        );
        ensure!(
            self.caster_spec == passive_caster_spec(),
            "caster spec drift"
        );
        ensure!(self.caster_spec.is_valid(), "invalid caster spec");
        ensure!(
            self.fixed_delta_ticks == DIFF_CASTER_FIXED_DELTA_TICKS,
            "step drift"
        );
        ensure!(
            self.seed == WORLD_SEED && self.steps == TOTAL_STEPS,
            "execution drift"
        );
        ensure!(
            self.samples.first().is_some_and(|sample| sample.step == 0)
                && self
                    .samples
                    .last()
                    .is_some_and(|sample| sample.step == TOTAL_STEPS),
            "sample boundaries"
        );
        ensure!(
            self.samples
                .windows(2)
                .all(|pair| pair[0].step < pair[1].step),
            "sample order"
        );
        for sample in &self.samples {
            ensure!(sample.is_finite(), "non-finite sample {}", sample.step);
            ensure!(
                sample.sim_time_ticks == sample.step * DIFF_CASTER_FIXED_DELTA_TICKS,
                "timestamp drift"
            );
            ensure!(
                sample.command_phase == phase_for_step(sample.step),
                "phase drift"
            );
            ensure!(
                sample.command_voltage_v == command_for_step(sample.step),
                "action drift"
            );
            ensure!(
                sample.drive_normal_load_n.iter().all(|load| *load >= 0.0)
                    && sample.caster_normal_load_n >= 0.0,
                "negative load"
            );
            ensure!(
                (0.0..=1.0).contains(&sample.caster_friction_utilization),
                "caster friction bound"
            );
            ensure!(
                sample.caster_swivel_rad.abs() <= std::f64::consts::PI,
                "wrapped caster angle bound"
            );
        }
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == trace_digest(self)?,
            "trace digest mismatch"
        );
        Ok(())
    }
}

/// Two complete backend traces with explicit SI-unit tolerance evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DifferentialCasterComparison {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// First backend trace.
    pub first: DifferentialCasterTrace,
    /// Second backend trace.
    pub second: DifferentialCasterTrace,
    /// Ordered cross-backend absolute gaps.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether all gaps pass.
    pub passed: bool,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl DifferentialCasterComparison {
    /// Recomputes trace integrity, common contract, tolerances, verdict, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == DIFF_CASTER_COMPARISON_KIND, "comparison kind");
        ensure!(
            self.schema_version == DIFF_CASTER_SCHEMA_VERSION,
            "comparison schema"
        );
        self.first.validate()?;
        self.second.validate()?;
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "distinct backends required"
        );
        ensure!(
            self.first.task_spec == self.second.task_spec
                && self.first.caster_spec == self.second.caster_spec,
            "backend contract mismatch"
        );
        ensure!(
            self.metrics == comparison_metrics(&self.first, &self.second)?,
            "metric drift"
        );
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == comparison_digest(self)?,
            "digest mismatch"
        );
        Ok(())
    }
}

/// Returns the exact open-loop arc/straighten/reverse task shared by all backends.
pub fn differential_caster_task_spec() -> TaskSpec {
    TaskSpec::new(
        DIFF_CASTER_TASK_ID,
        DIFF_CASTER_FIXED_DELTA_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![TensorSpec::new(
            "command_phase",
            TensorDType::U8,
            vec![],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(0.0, 3.0))]),
        ActionSpec::new(vec![TensorSpec::new(
            "left_right_motor_terminal_voltage_v",
            TensorDType::F64,
            vec![2],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(
            -COMMAND_VOLTAGE_V,
            COMMAND_VOLTAGE_V,
        ))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_forward_progress_m", 1.0, "m"),
            RewardTermSpec::new("truth_roll_pitch_limit", -1.0, "rad"),
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

/// Runs one explicit two-drive-wheel plus passive-caster multibody fixture.
pub fn run_differential_caster_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<DifferentialCasterTrace> {
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
    let task_spec = differential_caster_task_spec();
    task_spec.validate()?;
    let mut simulation = CasterPlant::new(backend)?;
    let plant = simulation.plant;
    let caster_spec = simulation.caster_spec;
    let initial_transform = *simulation
        .world
        .get::<Transform3>(simulation.chassis)
        .context("initial chassis")?;
    let mut drive_contact_steps = [0_u64; 2];
    let mut caster_contact_steps = 0_u64;
    let mut maximum_abs_caster_swivel_rad = 0.0_f64;
    let mut maximum_abs_caster_swivel_velocity_rad_s = 0.0_f64;
    let mut maximum_abs_caster_lateral_force_n = 0.0_f64;
    let mut minimum_accel_caster_load_n = f64::INFINITY;
    let mut maximum_brake_caster_load_n = 0.0_f64;
    let mut static_caster_load_sum_n = 0.0;
    let mut static_caster_load_samples = 0_u64;
    let mut samples = vec![make_sample(
        0,
        initial_transform,
        RigidBody::default(),
        0.0,
        [0.0; 2],
        0.0,
        false,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )];

    for zero_based_step in 0..TOTAL_STEPS {
        let step = zero_based_step + 1;
        let completed = simulation.step(command_for_step(step))?;
        let transform = completed.transform;
        let body = completed.body;
        let integrated_yaw_rad = completed.integrated_yaw_rad;
        let drive_loads_n = completed.drive_loads_n;
        let caster_in_contact = completed.caster_in_contact;
        let caster_swivel_rad = completed.caster_swivel_rad;
        let caster_swivel_unwrapped_rad = completed.caster_swivel_unwrapped_rad;
        let caster_swivel_velocity_rad_s = completed.caster_swivel_velocity_rad_s;
        let caster_roll_velocity_rad_s = completed.caster_roll_velocity_rad_s;
        if step > SETTLE_STEPS {
            for (count, in_contact) in drive_contact_steps
                .iter_mut()
                .zip(completed.drive_in_contact)
            {
                *count += u64::from(in_contact);
            }
            caster_contact_steps += u64::from(caster_in_contact);
        }
        maximum_abs_caster_swivel_rad = maximum_abs_caster_swivel_rad.max(caster_swivel_rad.abs());
        maximum_abs_caster_swivel_velocity_rad_s =
            maximum_abs_caster_swivel_velocity_rad_s.max(caster_swivel_velocity_rad_s.abs());
        maximum_abs_caster_lateral_force_n =
            maximum_abs_caster_lateral_force_n.max(completed.caster_lateral_force_n.abs());
        if (SETTLE_STEPS - 200..=SETTLE_STEPS).contains(&step) {
            static_caster_load_sum_n += completed.caster_load_n;
            static_caster_load_samples += 1;
        }
        if (ARC_END_STEP + 1..=ACCEL_END_STEP).contains(&step) {
            minimum_accel_caster_load_n = minimum_accel_caster_load_n.min(completed.caster_load_n);
        }
        if step > ACCEL_END_STEP {
            maximum_brake_caster_load_n = maximum_brake_caster_load_n.max(completed.caster_load_n);
        }

        if step % TRACE_STRIDE_STEPS == 0 || step == TOTAL_STEPS {
            samples.push(make_sample(
                step,
                transform,
                body,
                integrated_yaw_rad,
                drive_loads_n,
                completed.caster_load_n,
                caster_in_contact,
                caster_swivel_rad,
                caster_swivel_unwrapped_rad,
                caster_swivel_velocity_rad_s,
                caster_roll_velocity_rad_s,
                completed.caster_lateral_force_n,
                completed.caster_friction_utilization,
            ));
        }
    }

    let final_transform = *simulation
        .world
        .get::<Transform3>(simulation.chassis)
        .context("final chassis")?;
    let horizontal_displacement_m = (final_transform.translation.x
        - initial_transform.translation.x)
        .hypot(final_transform.translation.z - initial_transform.translation.z);
    let driven_duration = TOTAL_STEPS - SETTLE_STEPS;
    let static_caster_load_n = static_caster_load_sum_n / static_caster_load_samples as f64;
    let mut metrics = vec![
        metric(
            "caster_contact_fraction",
            "1",
            caster_contact_steps as f64 / driven_duration as f64,
            0.5,
            1.0,
        ),
        metric(
            "caster_load_recovery_n",
            "N",
            maximum_brake_caster_load_n - minimum_accel_caster_load_n,
            1.0,
            1_000.0,
        ),
        metric(
            "caster_unload_from_static_n",
            "N",
            static_caster_load_n - minimum_accel_caster_load_n,
            1.0,
            1_000.0,
        ),
        metric(
            "final_horizontal_displacement_m",
            "m",
            horizontal_displacement_m,
            0.1,
            20.0,
        ),
        metric(
            "maximum_absolute_caster_lateral_force_n",
            "N",
            maximum_abs_caster_lateral_force_n,
            1.0,
            1_000.0,
        ),
        metric(
            "maximum_absolute_caster_swivel_rad",
            "rad",
            maximum_abs_caster_swivel_rad,
            0.05,
            std::f64::consts::PI,
        ),
        metric(
            "maximum_absolute_caster_swivel_velocity_rad_s",
            "rad/s",
            maximum_abs_caster_swivel_velocity_rad_s,
            0.1,
            50.0,
        ),
        metric(
            "minimum_drive_contact_fraction",
            "1",
            drive_contact_steps
                .iter()
                .map(|steps| *steps as f64 / driven_duration as f64)
                .fold(1.0_f64, f64::min),
            0.5,
            1.0,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = DifferentialCasterTrace {
        kind: DIFF_CASTER_TRACE_KIND.to_string(),
        schema_version: DIFF_CASTER_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        drive_plant_spec: plant,
        drive_station_specs: drive_station_specs(),
        caster_spec,
        fixed_delta_ticks: DIFF_CASTER_FIXED_DELTA_TICKS,
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

// Shared plant state; controller-visible sensors are mounted by the caller.
// The original queued-wrench split is retained: apply previous forces, advance
// rigid bodies, then advance motor/tire state and queue the next forces.
pub(crate) struct CasterPlant<B> {
    backend: B,
    pub(crate) world: World,
    pub(crate) chassis: Entity,
    physics_world: rne_physics::PhysicsWorldId,
    pub(crate) plant: LongitudinalMobilityPlantSpec,
    caster_spec: PassiveCasterSpec,
    drive_stations: [DriveStation; 2],
    caster: PassiveCasterEntities,
    pub(crate) drive_states: [LongitudinalDrivePathState; 2],
    caster_tire_state: CombinedSlipTireState,
    conditioned_load_n: [f64; 3],
    pending_wrenches: Vec<ExternalBodyWrench>,
    integrated_yaw_rad: f64,
    step: u64,
}

impl<B> std::fmt::Debug for CasterPlant<B> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("CasterPlant")
            .field("step", &self.step)
            .field("chassis", &self.chassis)
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct CasterPlantStep {
    pub(crate) transform: Transform3,
    pub(crate) body: RigidBody,
    pub(crate) integrated_yaw_rad: f64,
    pub(crate) drive_loads_n: [f64; 2],
    pub(crate) drive_in_contact: [bool; 2],
    pub(crate) caster_load_n: f64,
    pub(crate) caster_in_contact: bool,
    pub(crate) caster_swivel_rad: f64,
    pub(crate) caster_swivel_unwrapped_rad: f64,
    pub(crate) caster_swivel_velocity_rad_s: f64,
    pub(crate) caster_roll_velocity_rad_s: f64,
    pub(crate) caster_lateral_force_n: f64,
    pub(crate) caster_friction_utilization: f64,
    pub(crate) motor_telemetry: [rne_robot::DcMotorCompletedTelemetry; 2],
}

impl<B: PhysicsBackend> CasterPlant<B> {
    pub(crate) fn new(mut backend: B) -> Result<Self> {
        let plant = drive_plant_spec();
        let caster_spec = passive_caster_spec();
        ensure!(caster_spec.is_valid(), "invalid caster fixture");
        let physics_world = backend.create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
            solver_iterations: 32,
        })?;
        let mut world = World::new();
        let ground = spawn_named(&mut world, "diff_caster_ground");
        world.entity_mut(ground).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            frictionless_cuboid(Vec3::new(20.0, 0.5, 20.0)),
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        ));
        let chassis = spawn_named(&mut world, "diff_caster_chassis");
        world.entity_mut(chassis).insert((
            RigidBody {
                mass_kg: 60.0,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::new(-0.03, 0.03, 0.0),
                ixx_kg_m2: 5.0,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 7.0,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 6.0,
            },
            MultibodyLink,
            frictionless_cuboid(Vec3::new(0.48, 0.10, 0.24)),
            CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.37, 0.0), Quat::IDENTITY),
        ));
        let drive_stations = spawn_drive_stations(&mut world, chassis);
        let caster = spawn_passive_caster(&mut world, chassis, caster_spec);
        backend.sync_from_ecs(&mut world, physics_world)?;

        Ok(Self {
            backend,
            world,
            chassis,
            physics_world,
            plant,
            caster_spec,
            drive_stations,
            caster,
            drive_states: [LongitudinalDrivePathState::default(); 2],
            caster_tire_state: CombinedSlipTireState::default(),
            conditioned_load_n: [0.0; 3],
            pending_wrenches: Vec::new(),
            integrated_yaw_rad: 0.0,
            step: 0,
        })
    }

    pub(crate) fn step(&mut self, commands: [f64; 2]) -> Result<CasterPlantStep> {
        ensure!(
            commands.iter().all(|v| v.is_finite()),
            "invalid terminal voltage"
        );
        let step = self.step.checked_add(1).context("caster step overflow")?;
        ensure!(
            step.checked_mul(DIFF_CASTER_FIXED_DELTA_TICKS).is_some(),
            "caster clock overflow"
        );
        let fixed_delta = SimDuration::from_ticks(DIFF_CASTER_FIXED_DELTA_TICKS);
        let dt_s = fixed_delta.as_seconds().value();
        let backend = &mut self.backend;
        let world = &mut self.world;
        let physics_world = self.physics_world;
        let chassis = self.chassis;
        let plant = self.plant;
        let caster_spec = self.caster_spec;
        let drive_stations = &self.drive_stations;
        let caster = &self.caster;
        let drive_states = &mut self.drive_states;
        let caster_tire_state = &mut self.caster_tire_state;
        let conditioned_load_n = &mut self.conditioned_load_n;
        let pending_wrenches = &mut self.pending_wrenches;
        let mut integrated_yaw_rad = self.integrated_yaw_rad;
        let mut drive_in_contact = [false; 2];
        let mut motor_telemetry = [rne_robot::DcMotorCompletedTelemetry::default(); 2];
        for wrench in pending_wrenches.drain(..) {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(world, physics_world)?;

        let transform = *world.get::<Transform3>(chassis).context("chassis pose")?;
        let body = *world.get::<RigidBody>(chassis).context("chassis body")?;
        integrated_yaw_rad += body.angular_velocity_rad_s.y * dt_s;
        let contacts = backend.contact_points(physics_world)?;
        let load_alpha = dt_s / (CONTACT_LOAD_FILTER_TIME_CONSTANT_S + dt_s);
        let mut drive_loads_n = [0.0; 2];

        for (index, station) in drive_stations.iter().enumerate() {
            let frame = resolve_wheel_station_frame(
                station.spec,
                0.0,
                transform,
                body.linear_velocity_m_s,
                body.angular_velocity_rad_s,
            )
            .with_context(|| format!("drive frame {index} at step {step}"))?;
            let raw_patch = aggregate_wheel_contact_patch(
                station.entity,
                contacts,
                frame.forward_world,
                frame.lateral_world,
            )
            .with_context(|| format!("drive contact {index} at step {step}"))?;
            drive_in_contact[index] = raw_patch.is_some();
            let bounded = raw_patch
                .map_or(0.0, |patch| patch.normal_load_n)
                .min(plant.tire.reference_load_n * plant.tire.maximum_load_ratio);
            conditioned_load_n[index] += load_alpha * (bounded - conditioned_load_n[index]);
            drive_loads_n[index] = conditioned_load_n[index];
            let patch = raw_patch.map(|mut patch| {
                patch.normal_load_n = conditioned_load_n[index];
                patch
            });
            let evaluation = evaluate_longitudinal_drive_path(
                plant,
                drive_states[index],
                LongitudinalDrivePathInput {
                    carrier_patch: patch,
                    forward_world: frame.forward_world,
                    lateral_world: frame.lateral_world,
                    command_voltage_v: commands[index],
                },
                dt_s,
            )
            .with_context(|| format!("drive path {index} at step {step}"))?;
            drive_states[index] = evaluation.state;
            motor_telemetry[index] = evaluation.motor_telemetry;
            if let Some(wrench) = evaluation.tire_wrench {
                pending_wrenches.push(wrench);
            }
        }

        let fork_transform = *world
            .get::<Transform3>(caster.bracket)
            .context("caster bracket")?;
        let caster_forward = (fork_transform.rotation * Vec3::X).normalize();
        let caster_lateral = (fork_transform.rotation * Vec3::Z).normalize();
        let raw_caster_patch =
            aggregate_wheel_contact_patch(caster.wheel, contacts, caster_forward, caster_lateral)
                .with_context(|| format!("caster contact at step {step}"))?;
        let caster_in_contact = raw_caster_patch.is_some();
        let bounded_caster_load = raw_caster_patch
            .map_or(0.0, |patch| patch.normal_load_n)
            .min(caster_spec.tire.reference_load_n * caster_spec.tire.maximum_load_ratio);
        conditioned_load_n[2] += load_alpha * (bounded_caster_load - conditioned_load_n[2]);
        let caster_patch = raw_caster_patch.map(|mut patch| {
            patch.normal_load_n = conditioned_load_n[2];
            patch
        });
        let caster_roll_state = *world
            .get::<JointState>(caster.wheel)
            .context("caster roll")?;
        let caster_roll_velocity_rad_s = match caster_roll_state {
            JointState::Revolute { velocity_rad_s, .. } => velocity_rad_s,
            _ => anyhow::bail!("caster roll joint state kind"),
        };
        let caster_tire = evaluate_combined_slip_tire(
            caster_spec.tire,
            *caster_tire_state,
            CombinedSlipTireInput {
                patch: caster_patch,
                forward_world: caster_forward,
                lateral_world: caster_lateral,
                wheel_circumferential_speed_m_s: caster_roll_velocity_rad_s
                    * caster_spec.wheel_radius_m,
                road_friction_scale: 1.0,
            },
            dt_s,
        )
        .with_context(|| format!("caster tire at step {step}"))?;
        *caster_tire_state = caster_tire.state;
        if let Some(patch) = caster_patch {
            pending_wrenches.push(
                combined_slip_tire_wrench(patch, caster_tire, caster_forward, caster_lateral)
                    .with_context(|| format!("caster wrench at step {step}"))?,
            );
        }
        let caster_swivel_state = *world
            .get::<JointState>(caster.bracket)
            .context("caster swivel")?;
        let (caster_swivel_unwrapped_rad, caster_swivel_velocity_rad_s) = match caster_swivel_state
        {
            JointState::Revolute {
                position_rad,
                velocity_rad_s,
            } => (position_rad, velocity_rad_s),
            _ => anyhow::bail!("caster swivel joint state kind"),
        };
        let caster_swivel_rad = wrap_angle_rad(caster_swivel_unwrapped_rad);
        self.step = step;
        self.integrated_yaw_rad = integrated_yaw_rad;
        Ok(CasterPlantStep {
            transform,
            body,
            integrated_yaw_rad,
            drive_loads_n,
            drive_in_contact,
            caster_load_n: conditioned_load_n[2],
            caster_in_contact,
            caster_swivel_rad,
            caster_swivel_unwrapped_rad,
            caster_swivel_velocity_rad_s,
            caster_roll_velocity_rad_s,
            caster_lateral_force_n: caster_tire.lateral_force_n,
            caster_friction_utilization: caster_tire.friction_utilization,
            motor_telemetry,
        })
    }
}

/// Builds a self-verifying SI-unit comparison from two complete backend traces.
pub fn compare_differential_caster_traces(
    first: DifferentialCasterTrace,
    second: DifferentialCasterTrace,
) -> Result<DifferentialCasterComparison> {
    first.validate()?;
    second.validate()?;
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = DifferentialCasterComparison {
        kind: DIFF_CASTER_COMPARISON_KIND.to_string(),
        schema_version: DIFF_CASTER_SCHEMA_VERSION,
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

#[derive(Clone, Copy, Debug)]
struct DriveStation {
    entity: Entity,
    spec: WheelStationSpec,
}

#[derive(Clone, Copy, Debug)]
struct PassiveCasterEntities {
    bracket: Entity,
    wheel: Entity,
}

fn spawn_drive_stations(world: &mut World, chassis: Entity) -> [DriveStation; 2] {
    let specs = drive_station_specs();
    std::array::from_fn(|index| {
        let spec = specs[index];
        let entity = spawn_named(
            world,
            if index == 0 {
                "drive_left"
            } else {
                "drive_right"
            },
        );
        world.entity_mut(entity).insert((
            RigidBody {
                mass_kg: 5.0,
                ..RigidBody::default()
            },
            frictionless_sphere(DRIVE_WHEEL_RADIUS_M),
            CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
            Transform3::from_translation_rotation(
                Vec3::new(
                    spec.center_body_m.x,
                    0.37 + spec.center_body_m.y,
                    spec.center_body_m.z,
                ),
                Quat::IDENTITY,
            ),
            FixedJointDesc {
                parent: chassis,
                anchor_parent_m: spec.center_body_m,
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
            },
            spec,
        ));
        DriveStation { entity, spec }
    })
}

fn spawn_passive_caster(
    world: &mut World,
    chassis: Entity,
    spec: PassiveCasterSpec,
) -> PassiveCasterEntities {
    let bracket = spawn_named(world, "caster_swivel_bracket");
    let mount_world_m = Vec3::new(
        spec.mount_body_m.x,
        0.37 + spec.mount_body_m.y,
        spec.mount_body_m.z,
    );
    let initial_rotation = Quat::from_rotation_y(spec.initial_swivel_rad);
    world.entity_mut(bracket).insert((
        RigidBody {
            mass_kg: spec.bracket_mass_kg,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::new(-0.5 * spec.trail_m, 0.0, 0.0),
            ixx_kg_m2: spec.swivel_inertia_kg_m2,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: spec.swivel_inertia_kg_m2,
            iyz_kg_m2: 0.0,
            izz_kg_m2: spec.swivel_inertia_kg_m2,
        },
        MultibodyLink,
        Transform3::from_translation_rotation(mount_world_m, initial_rotation),
        RevoluteJointDesc {
            parent: chassis,
            axis: Vec3::Y,
            anchor_parent_m: spec.mount_body_m,
            anchor_child_m: Vec3::ZERO,
            lower_rad: None,
            upper_rad: None,
            relative_rotation: Quat::IDENTITY,
        },
        JointState::Revolute {
            position_rad: spec.initial_swivel_rad,
            velocity_rad_s: 0.0,
        },
        JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: spec.swivel_damping_nm_s_per_rad,
            coulomb_friction_nm: 0.0,
            coulomb_transition_velocity_rad_s: 0.0,
        },
        spec,
    ));
    let wheel = spawn_named(world, "caster_wheel");
    let axle_offset_world = initial_rotation * Vec3::new(-spec.trail_m, 0.0, 0.0);
    world.entity_mut(wheel).insert((
        RigidBody {
            mass_kg: spec.wheel_mass_kg,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: spec.wheel_axle_inertia_kg_m2 * 0.6,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: spec.wheel_axle_inertia_kg_m2 * 0.6,
            iyz_kg_m2: 0.0,
            izz_kg_m2: spec.wheel_axle_inertia_kg_m2,
        },
        MultibodyLink,
        frictionless_sphere(spec.wheel_radius_m),
        CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
        Transform3::from_translation_rotation(mount_world_m + axle_offset_world, initial_rotation),
        RevoluteJointDesc {
            parent: bracket,
            // Positive joint speed must produce positive circumference speed
            // along the caster's +X forward axis at the -Y contact point.
            axis: -Vec3::Z,
            anchor_parent_m: Vec3::new(-spec.trail_m, 0.0, 0.0),
            anchor_child_m: Vec3::ZERO,
            lower_rad: None,
            upper_rad: None,
            relative_rotation: Quat::IDENTITY,
        },
        JointState::Revolute {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
        },
        JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: spec.rolling_damping_nm_s_per_rad,
            coulomb_friction_nm: 0.0,
            coulomb_transition_velocity_rad_s: 0.0,
        },
    ));
    PassiveCasterEntities { bracket, wheel }
}

fn drive_station_specs() -> [WheelStationSpec; 2] {
    [0.30, -0.30].map(|z| WheelStationSpec {
        center_body_m: Vec3::new(-0.22, -0.25, z),
        ..WheelStationSpec::default()
    })
}

fn passive_caster_spec() -> PassiveCasterSpec {
    PassiveCasterSpec {
        mount_body_m: Vec3::new(0.36, -0.25, 0.0),
        trail_m: 0.09,
        wheel_radius_m: 0.12,
        bracket_mass_kg: 3.0,
        wheel_mass_kg: 4.0,
        swivel_inertia_kg_m2: 0.06,
        wheel_axle_inertia_kg_m2: 0.03,
        swivel_damping_nm_s_per_rad: 0.8,
        rolling_damping_nm_s_per_rad: 0.03,
        initial_swivel_rad: 0.0,
        tire: CombinedSlipTireSpec {
            reference_load_n: 260.0,
            longitudinal_stiffness_n: 500.0,
            lateral_stiffness_n: 600.0,
            longitudinal_peak_friction: 0.5,
            lateral_peak_friction: 0.5,
            longitudinal_relaxation_length_m: 0.05,
            lateral_relaxation_length_m: 0.08,
            ..CombinedSlipTireSpec::default()
        },
    }
}

fn drive_plant_spec() -> LongitudinalMobilityPlantSpec {
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: 75.0,
        driven_wheel_count: 1,
        normal_load_per_driven_wheel_n: 240.0,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.0,
        road_friction_scale: 1.0,
        motor: DcMotorSpec::default(),
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec {
            radius_m: DRIVE_WHEEL_RADIUS_M,
            width_m: 0.05,
            inertia_kg_m2: 0.02,
            ..WheelAssemblySpec::default()
        },
        tire: CombinedSlipTireSpec {
            reference_load_n: 240.0,
            longitudinal_stiffness_n: 1_500.0,
            lateral_stiffness_n: 2_000.0,
            longitudinal_peak_friction: 0.6,
            lateral_peak_friction: 0.6,
            longitudinal_relaxation_length_m: 0.03,
            lateral_relaxation_length_m: 0.05,
            ..CombinedSlipTireSpec::default()
        },
        longitudinal_load_transfer: None,
    }
}

fn phase_for_step(step: u64) -> u8 {
    if step <= SETTLE_STEPS {
        0
    } else if step <= ARC_END_STEP {
        1
    } else if step <= ACCEL_END_STEP {
        2
    } else {
        3
    }
}

fn command_for_step(step: u64) -> [f64; 2] {
    match phase_for_step(step) {
        0 => [0.0, 0.0],
        1 => {
            let ramp = ((step - SETTLE_STEPS) as f64 / 400.0).min(1.0);
            [ramp * 0.375 * COMMAND_VOLTAGE_V, ramp * COMMAND_VOLTAGE_V]
        }
        2 => {
            let blend = ((step - ARC_END_STEP) as f64 / 400.0).min(1.0);
            [
                (0.375 + 0.625 * blend) * COMMAND_VOLTAGE_V,
                COMMAND_VOLTAGE_V,
            ]
        }
        _ => {
            let blend = ((step - ACCEL_END_STEP) as f64 / 600.0).min(1.0);
            let voltage = (1.0 - 2.0 * blend) * COMMAND_VOLTAGE_V;
            [voltage, voltage]
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn make_sample(
    step: u64,
    transform: Transform3,
    body: RigidBody,
    integrated_yaw_rad: f64,
    drive_normal_load_n: [f64; 2],
    caster_normal_load_n: f64,
    caster_in_contact: bool,
    caster_swivel_rad: f64,
    caster_swivel_unwrapped_rad: f64,
    caster_swivel_velocity_rad_s: f64,
    caster_roll_velocity_rad_s: f64,
    caster_lateral_force_n: f64,
    caster_friction_utilization: f64,
) -> DifferentialCasterSample {
    DifferentialCasterSample {
        step,
        sim_time_ticks: SimTime::from_ticks(step * DIFF_CASTER_FIXED_DELTA_TICKS).ticks(),
        command_phase: phase_for_step(step),
        command_voltage_v: command_for_step(step),
        privileged_position_world_m: transform.translation.to_array().map(f64::from),
        privileged_integrated_yaw_rad: integrated_yaw_rad,
        privileged_forward_velocity_m_s: body.linear_velocity_m_s.dot(transform.rotation * Vec3::X),
        drive_normal_load_n,
        caster_normal_load_n,
        caster_in_contact,
        caster_swivel_rad,
        caster_swivel_unwrapped_rad,
        caster_swivel_velocity_rad_s,
        caster_roll_velocity_rad_s,
        caster_lateral_force_n,
        caster_friction_utilization,
    }
}

fn wrap_angle_rad(angle_rad: f64) -> f64 {
    (angle_rad + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn frictionless_cuboid(half_extents_m: Vec3) -> Collider {
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
        passed: value >= minimum && value <= maximum,
    }
}

fn validate_metrics(metrics: &[MobilityBenchmarkMetric], passed: bool) -> Result<()> {
    ensure!(!metrics.is_empty(), "metrics omitted");
    ensure!(
        metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
        "metric order"
    );
    ensure!(
        metrics.iter().all(|metric| metric.value.is_finite()
            && metric.minimum.is_finite()
            && metric.maximum.is_finite()
            && metric.minimum <= metric.maximum
            && metric.passed == (metric.value >= metric.minimum && metric.value <= metric.maximum)),
        "metric drift"
    );
    ensure!(
        passed == metrics.iter().all(|metric| metric.passed),
        "verdict drift"
    );
    Ok(())
}

fn comparison_metrics(
    first: &DifferentialCasterTrace,
    second: &DifferentialCasterTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    let mut metrics = vec![
        gap_metric(
            "caster_load_recovery_gap_n",
            "N",
            first,
            second,
            "caster_load_recovery_n",
            200.0,
        )?,
        gap_metric(
            "caster_swivel_gap_rad",
            "rad",
            first,
            second,
            "maximum_absolute_caster_swivel_rad",
            1.0,
        )?,
        gap_metric(
            "displacement_gap_m",
            "m",
            first,
            second,
            "final_horizontal_displacement_m",
            1.0,
        )?,
        gap_metric(
            "lateral_force_gap_n",
            "N",
            first,
            second,
            "maximum_absolute_caster_lateral_force_n",
            300.0,
        )?,
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    Ok(metrics)
}

fn gap_metric(
    id: &str,
    unit: &str,
    first: &DifferentialCasterTrace,
    second: &DifferentialCasterTrace,
    source: &str,
    maximum: f64,
) -> Result<MobilityBenchmarkMetric> {
    let value = (metric_value(first, source)? - metric_value(second, source)?).abs();
    Ok(metric(id, unit, value, 0.0, maximum))
}

fn metric_value(trace: &DifferentialCasterTrace, id: &str) -> Result<f64> {
    trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .map(|metric| metric.value)
        .with_context(|| format!("missing metric {id}"))
}

fn trace_digest(trace: &DifferentialCasterTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn comparison_digest(comparison: &DifferentialCasterComparison) -> Result<String> {
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
    fn passive_caster_contract_rejects_zero_trail() {
        let mut spec = passive_caster_spec();
        spec.trail_m = 0.0;
        assert!(!spec.is_valid());
    }

    #[test]
    fn rapier_diff_caster_trace_is_passing_and_deterministic() {
        let first =
            run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let second =
            run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert!(first.passed, "{:#?}", first.metrics);
        assert_eq!(first, second);
        first.validate().unwrap();
    }

    #[test]
    fn diff_caster_trace_tampering_is_detected() {
        let mut trace =
            run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace.samples[1].caster_swivel_rad += 0.1;
        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_diff_caster_traces_pass() {
        use rne_physics_mujoco::MuJoCoBackend;
        let rapier =
            run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_differential_caster_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(DIFF_CASTER_FIXED_DELTA_TICKS)).unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_differential_caster_traces(rapier, mujoco).unwrap();
        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();
    }
}
