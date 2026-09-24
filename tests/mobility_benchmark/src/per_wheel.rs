//! Per-wheel skid-steer contact, drive-path, and yaw execution across physics backends.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{y_up_euler_rad, Quat, Vec3};
use rne_physics::{
    require_capabilities, Collider, CollisionGroups, ExternalBodyWrench, FixedJointDesc,
    PhysicsBackend, PhysicsBackendManifest, PhysicsCapability, PhysicsMaterial, PhysicsWorldDesc,
    RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, resolve_wheel_station_frame,
    CombinedSlipTireSpec, DcMotorSpec, LongitudinalDrivePathInput, LongitudinalDrivePathState,
    LongitudinalMobilityPlantSpec, TransmissionSpec, WheelAssemblySpec, WheelStationSpec,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

use crate::MobilityBenchmarkMetric;

/// Artifact discriminator for one per-wheel skid trace.
pub const PER_WHEEL_SKID_TRACE_KIND: &str = "rne_mobility_per_wheel_skid_trace";
/// Artifact discriminator for a per-wheel skid cross-backend comparison.
pub const PER_WHEEL_SKID_COMPARISON_KIND: &str = "rne_mobility_per_wheel_skid_comparison";
/// Current per-wheel artifact schema version.
pub const PER_WHEEL_SKID_SCHEMA_VERSION: u32 = 1;
/// Stable `TaskSpec` identity shared by every backend run.
pub const PER_WHEEL_SKID_TASK_ID: &str = "mobility_per_wheel_skid_pivot_v1";
/// Fixed physics and drive-path step in simulation nanosecond ticks.
pub const PER_WHEEL_SKID_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SETTLE_STEPS: u64 = 500;
const DRIVE_STEPS: u64 = 2_000;
const TOTAL_STEPS: u64 = SETTLE_STEPS + DRIVE_STEPS;
const TRACE_STRIDE_STEPS: u64 = 100;
const COMMAND_VOLTAGE_V: f64 = 12.0;
const WORLD_SEED: u64 = 0;
pub(crate) const VEHICLE_MASS_KG: f64 = 100.0;
const WHEEL_RADIUS_M: f64 = 0.12;
const SELF_COLLISION_GROUP: u32 = 1;
pub(crate) const CONTACT_LOAD_FILTER_TIME_CONSTANT_S: f64 = 0.02;

/// One downsampled per-wheel row; chassis fields are explicitly privileged truth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelSkidSample {
    /// One-based completed physics step, or zero for the initial state.
    pub step: u64,
    /// Completed simulation time in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Task-provided actor observation, zero while settling and one while driving.
    pub command_phase: f64,
    /// Left and right `TaskSpec` actions in volts.
    pub command_voltage_v: [f64; 2],
    /// Privileged chassis position in world coordinates, in meters.
    pub privileged_position_world_m: [f64; 3],
    /// Privileged unwrapped yaw accumulated from backend angular velocity, in radians.
    pub privileged_integrated_yaw_rad: f64,
    /// Privileged wrapped chassis yaw, in radians.
    pub privileged_wrapped_yaw_rad: f64,
    /// Privileged world-Y yaw rate, in radians per second.
    pub privileged_yaw_rate_rad_s: f64,
    /// Per-wheel completed normal load in `[front_left, rear_left, front_right, rear_right]`, N.
    pub wheel_normal_load_n: [f64; 4],
    /// Per-wheel raw solver normal load before tire-bandwidth conditioning, in newtons.
    pub wheel_raw_normal_load_n: [f64; 4],
    /// Per-wheel rotational velocity in radians per second.
    pub wheel_velocity_rad_s: [f64; 4],
    /// Per-wheel longitudinal tire force scheduled for the next step, in newtons.
    pub wheel_longitudinal_force_n: [f64; 4],
    /// Per-wheel lateral scrub force scheduled for the next step, in newtons.
    pub wheel_lateral_force_n: [f64; 4],
    /// Per-wheel combined-friction utilization.
    pub wheel_friction_utilization: [f64; 4],
}

impl PerWheelSkidSample {
    fn is_finite(&self) -> bool {
        self.command_phase.is_finite()
            && self
                .command_voltage_v
                .iter()
                .chain(self.privileged_position_world_m.iter())
                .chain(self.wheel_normal_load_n.iter())
                .chain(self.wheel_raw_normal_load_n.iter())
                .chain(self.wheel_velocity_rad_s.iter())
                .chain(self.wheel_longitudinal_force_n.iter())
                .chain(self.wheel_lateral_force_n.iter())
                .chain(self.wheel_friction_utilization.iter())
                .all(|value| value.is_finite())
            && self.privileged_integrated_yaw_rad.is_finite()
            && self.privileged_wrapped_yaw_rad.is_finite()
            && self.privileged_yaw_rate_rad_s.is_finite()
    }
}

/// Deterministic four-wheel skid-steer trace from one rigid-body backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelSkidTrace {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// Exact backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact portable task executed by this run.
    pub task_spec: TaskSpec,
    /// Exact motor, transmission, wheel, tire, and road force-element profile.
    pub plant_spec: LongitudinalMobilityPlantSpec,
    /// Ordered physical station geometry in sample wheel order.
    pub wheel_station_specs: [WheelStationSpec; 4],
    /// First-order bandwidth applied between raw solver load and tire load, in seconds.
    pub contact_load_filter_time_constant_s: f64,
    /// Fixed step in simulation nanosecond ticks.
    pub fixed_delta_ticks: u64,
    /// Explicit world seed.
    pub seed: u64,
    /// Number of completed physics steps.
    pub steps: u64,
    /// Ordered, downsampled, unit-bearing evidence.
    pub samples: Vec<PerWheelSkidSample>,
    /// Ordered acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every acceptance metric passed.
    pub passed: bool,
    /// FNV-1a digest of the same trace with this field empty.
    pub content_digest: String,
}

impl PerWheelSkidTrace {
    /// Recomputes the task, ordering, metric verdicts, and content digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == PER_WHEEL_SKID_TRACE_KIND, "kind mismatch");
        ensure!(
            self.schema_version == PER_WHEEL_SKID_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.backend.validate().context("backend manifest")?;
        self.task_spec.validate().context("TaskSpec")?;
        ensure!(
            self.task_spec == per_wheel_skid_task_spec(),
            "exact TaskSpec mismatch"
        );
        ensure!(self.plant_spec == wheel_plant_spec(), "plant spec mismatch");
        ensure!(
            self.wheel_station_specs == wheel_station_specs(),
            "wheel station geometry mismatch"
        );
        ensure!(
            self.contact_load_filter_time_constant_s == CONTACT_LOAD_FILTER_TIME_CONSTANT_S,
            "contact load bandwidth mismatch"
        );
        ensure!(
            self.fixed_delta_ticks == PER_WHEEL_SKID_FIXED_DELTA_TICKS,
            "fixed step mismatch"
        );
        ensure!(self.seed == WORLD_SEED, "seed mismatch");
        ensure!(self.steps == TOTAL_STEPS, "step count mismatch");
        ensure!(!self.samples.is_empty(), "trace omitted samples");
        ensure!(
            self.samples.first().is_some_and(|sample| sample.step == 0)
                && self
                    .samples
                    .last()
                    .is_some_and(|sample| sample.step == TOTAL_STEPS),
            "trace boundary samples mismatch"
        );
        ensure!(
            self.samples
                .windows(2)
                .all(|pair| pair[0].step < pair[1].step),
            "sample steps are not strictly ordered"
        );
        for sample in &self.samples {
            ensure!(sample.is_finite(), "sample {} is non-finite", sample.step);
            ensure!(
                sample.sim_time_ticks == sample.step * self.fixed_delta_ticks,
                "sample {} timestamp mismatch",
                sample.step
            );
            let phase = if sample.step > SETTLE_STEPS { 1.0 } else { 0.0 };
            ensure!(sample.command_phase == phase, "command phase mismatch");
            ensure!(
                sample.command_voltage_v == [phase * COMMAND_VOLTAGE_V, -phase * COMMAND_VOLTAGE_V],
                "command action mismatch"
            );
            ensure!(
                sample
                    .wheel_normal_load_n
                    .iter()
                    .all(|load_n| *load_n >= 0.0),
                "negative wheel load"
            );
            ensure!(
                sample
                    .wheel_raw_normal_load_n
                    .iter()
                    .all(|load_n| *load_n >= 0.0),
                "negative raw wheel load"
            );
            ensure!(
                sample
                    .wheel_friction_utilization
                    .iter()
                    .all(|value| (0.0..=1.0).contains(value)),
                "friction utilization escaped bounds"
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

/// Two complete per-wheel traces plus explicit SI-unit backend tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelSkidComparison {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// First complete backend trace.
    pub first: PerWheelSkidTrace,
    /// Second complete backend trace.
    pub second: PerWheelSkidTrace,
    /// Ordered cross-backend absolute-gap metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every gap is inside tolerance.
    pub passed: bool,
    /// FNV-1a digest of the same comparison with this field empty.
    pub content_digest: String,
}

impl PerWheelSkidComparison {
    /// Recomputes trace integrity, shared execution contract, metrics, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == PER_WHEEL_SKID_COMPARISON_KIND,
            "comparison kind mismatch"
        );
        ensure!(
            self.schema_version == PER_WHEEL_SKID_SCHEMA_VERSION,
            "comparison schema mismatch"
        );
        self.first.validate().context("first trace")?;
        self.second.validate().context("second trace")?;
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "comparison requires distinct backends"
        );
        ensure!(
            self.first.task_spec == self.second.task_spec
                && self.first.fixed_delta_ticks == self.second.fixed_delta_ticks
                && self.first.seed == self.second.seed,
            "backend execution contract mismatch"
        );
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

/// Returns the exact open-loop pivot task shared by all physics backends.
pub fn per_wheel_skid_task_spec() -> TaskSpec {
    TaskSpec::new(
        PER_WHEEL_SKID_TASK_ID,
        PER_WHEEL_SKID_FIXED_DELTA_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![TensorSpec::new(
            "command_phase",
            TensorDType::F64,
            vec![],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(0.0, 1.0))]),
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
            RewardTermSpec::new("truth_absolute_yaw_rad", 1.0, "rad"),
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

/// Runs a four-station skid pivot with independent motor, wheel, and tire states.
#[allow(clippy::too_many_lines)] // TODO(cleanup): split (274/150 lines); see PR body
pub fn run_per_wheel_skid_trace<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<PerWheelSkidTrace> {
    manifest.validate().context("backend manifest")?;
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::ContactForce,
            PhysicsCapability::ContactPointKinematics,
            PhysicsCapability::ExternalBodyWrench,
            PhysicsCapability::Articulation,
        ],
    )?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "backend manifest capability drift"
    );
    let task_spec = per_wheel_skid_task_spec();
    task_spec.validate().context("TaskSpec")?;
    let fixed_delta = SimDuration::from_ticks(PER_WHEEL_SKID_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 24,
    })?;
    let mut world = World::new();
    let ground = spawn_named(&mut world, "per_wheel_skid_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        frictionless_cuboid(Vec3::new(20.0, 0.5, 20.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let chassis = spawn_named(&mut world, "per_wheel_skid_chassis");
    world.entity_mut(chassis).insert((
        RigidBody {
            mass_kg: 80.0,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: 8.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 12.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 10.0,
        },
        frictionless_cuboid(Vec3::new(0.5, 0.12, 0.24)),
        CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.37, 0.0), Quat::IDENTITY),
    ));
    let stations = spawn_wheel_stations(&mut world, chassis);
    backend.sync_from_ecs(&mut world, physics_world)?;

    let plant = wheel_plant_spec();
    let mut states = [LongitudinalDrivePathState::default(); 4];
    let mut pending_wrenches: Vec<ExternalBodyWrench> = Vec::new();
    let initial_transform = *world
        .get::<Transform3>(chassis)
        .context("initial chassis pose")?;
    let mut integrated_yaw_rad = 0.0;
    let mut wheel_contact_drive_steps = [0_u64; 4];
    let mut conditioned_load_n = [0.0_f64; 4];
    let mut maximum_abs_yaw_rate_rad_s = 0.0_f64;
    let mut maximum_total_scrub_force_n = 0.0_f64;
    let mut maximum_wheel_load_spread_n = 0.0_f64;
    let mut maximum_utilization = 0.0_f64;
    let mut samples = vec![sample(
        0,
        0.0,
        [0.0, 0.0],
        initial_transform,
        0.0,
        RigidBody::default(),
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
    )];

    for zero_based_step in 0..TOTAL_STEPS {
        for wrench in pending_wrenches.drain(..) {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;

        let transform = *world.get::<Transform3>(chassis).context("chassis pose")?;
        let body = *world.get::<RigidBody>(chassis).context("chassis body")?;
        integrated_yaw_rad += body.angular_velocity_rad_s.y * dt_s;
        maximum_abs_yaw_rate_rad_s =
            maximum_abs_yaw_rate_rad_s.max(body.angular_velocity_rad_s.y.abs());
        let phase = if zero_based_step >= SETTLE_STEPS {
            1.0
        } else {
            0.0
        };
        let side_commands = [phase * COMMAND_VOLTAGE_V, -phase * COMMAND_VOLTAGE_V];
        let mut loads_n = [0.0; 4];
        let mut raw_loads_n = [0.0; 4];
        let mut wheel_velocity_rad_s = [0.0; 4];
        let mut longitudinal_force_n = [0.0; 4];
        let mut lateral_force_n = [0.0; 4];
        let mut utilization = [0.0; 4];
        let contacts = backend.contact_points(physics_world)?;
        for (index, station) in stations.iter().enumerate() {
            let frame = resolve_wheel_station_frame(
                station.spec,
                0.0,
                transform,
                body.linear_velocity_m_s,
                body.angular_velocity_rad_s,
            )?;
            let raw_patch = aggregate_wheel_contact_patch(
                station.entity,
                contacts,
                frame.forward_world,
                frame.lateral_world,
            )?;
            raw_loads_n[index] = raw_patch.map_or(0.0, |value| value.normal_load_n);
            if zero_based_step >= SETTLE_STEPS && raw_patch.is_some() {
                wheel_contact_drive_steps[index] += 1;
            }
            let maximum_valid_load_n = plant.tire.reference_load_n * plant.tire.maximum_load_ratio;
            let bounded_raw_load_n = raw_loads_n[index].min(maximum_valid_load_n);
            let load_alpha = dt_s / (CONTACT_LOAD_FILTER_TIME_CONSTANT_S + dt_s);
            conditioned_load_n[index] +=
                load_alpha * (bounded_raw_load_n - conditioned_load_n[index]);
            loads_n[index] = conditioned_load_n[index];
            let patch = raw_patch.map(|mut value| {
                value.normal_load_n = conditioned_load_n[index];
                value
            });
            let command_voltage_v = if station.spec.center_body_m.z > 0.0 {
                side_commands[0]
            } else {
                side_commands[1]
            };
            let evaluation = evaluate_longitudinal_drive_path(
                plant,
                states[index],
                LongitudinalDrivePathInput {
                    carrier_patch: patch,
                    forward_world: frame.forward_world,
                    lateral_world: frame.lateral_world,
                    command_voltage_v,
                },
                dt_s,
            )?;
            states[index] = evaluation.state;
            if let Some(mut wrench) = evaluation.tire_wrench {
                // The fixed station carries contact identity, while its force is
                // transmitted to the owning free chassis at the same world point.
                // This is physically equivalent for a rigid station and avoids
                // depending on whether a backend represents the weld as a solver
                // constraint (Rapier) or a fused child body (MuJoCo).
                wrench.entity = chassis;
                pending_wrenches.push(wrench);
            }
            wheel_velocity_rad_s[index] = evaluation.state.wheel_velocity_rad_s;
            longitudinal_force_n[index] = evaluation.tire.longitudinal_force_n;
            lateral_force_n[index] = evaluation.tire.lateral_force_n;
            utilization[index] = evaluation.tire.friction_utilization;
            maximum_utilization = maximum_utilization.max(evaluation.tire.friction_utilization);
        }
        maximum_total_scrub_force_n = maximum_total_scrub_force_n
            .max(lateral_force_n.iter().map(|force_n| force_n.abs()).sum());
        let minimum_load_n = loads_n.iter().copied().fold(f64::INFINITY, f64::min);
        let maximum_load_n = loads_n.iter().copied().fold(0.0_f64, f64::max);
        maximum_wheel_load_spread_n =
            maximum_wheel_load_spread_n.max(maximum_load_n - minimum_load_n);

        let step = zero_based_step + 1;
        if step % TRACE_STRIDE_STEPS == 0 || step == TOTAL_STEPS {
            samples.push(sample(
                step,
                phase,
                side_commands,
                transform,
                integrated_yaw_rad,
                body,
                loads_n,
                raw_loads_n,
                wheel_velocity_rad_s,
                longitudinal_force_n,
                lateral_force_n,
                utilization,
            ));
        }
    }

    let final_transform = *world
        .get::<Transform3>(chassis)
        .context("final chassis pose")?;
    let final_body = *world
        .get::<RigidBody>(chassis)
        .context("final chassis body")?;
    let horizontal_displacement_m = (final_transform.translation.x
        - initial_transform.translation.x)
        .hypot(final_transform.translation.z - initial_transform.translation.z);
    let mut metrics = vec![
        metric(
            "minimum_wheel_contact_drive_fraction",
            "1",
            wheel_contact_drive_steps
                .iter()
                .map(|steps| *steps as f64 / DRIVE_STEPS as f64)
                .fold(1.0_f64, f64::min),
            0.5,
            1.0,
        ),
        metric(
            "final_absolute_integrated_yaw_rad",
            "rad",
            integrated_yaw_rad.abs(),
            0.2,
            6.0,
        ),
        metric(
            "final_horizontal_displacement_m",
            "m",
            horizontal_displacement_m,
            0.0,
            0.5,
        ),
        metric(
            "final_absolute_yaw_rate_rad_s",
            "rad/s",
            final_body.angular_velocity_rad_s.y.abs(),
            0.05,
            5.0,
        ),
        metric(
            "maximum_absolute_yaw_rate_rad_s",
            "rad/s",
            maximum_abs_yaw_rate_rad_s,
            0.1,
            5.0,
        ),
        metric(
            "maximum_total_lateral_scrub_force_n",
            "N",
            maximum_total_scrub_force_n,
            1.0,
            2_000.0,
        ),
        metric(
            "maximum_wheel_load_spread_n",
            "N",
            maximum_wheel_load_spread_n,
            0.0,
            VEHICLE_MASS_KG * 9.806_65,
        ),
        metric(
            "maximum_wheel_tire_utilization",
            "1",
            maximum_utilization,
            0.01,
            1.0,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = PerWheelSkidTrace {
        kind: PER_WHEEL_SKID_TRACE_KIND.to_string(),
        schema_version: PER_WHEEL_SKID_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        plant_spec: plant,
        wheel_station_specs: stations.map(|station| station.spec),
        contact_load_filter_time_constant_s: CONTACT_LOAD_FILTER_TIME_CONSTANT_S,
        fixed_delta_ticks: PER_WHEEL_SKID_FIXED_DELTA_TICKS,
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

/// Builds and verifies an SI-unit per-wheel cross-backend comparison.
pub fn compare_per_wheel_skid_traces(
    first: PerWheelSkidTrace,
    second: PerWheelSkidTrace,
) -> Result<PerWheelSkidComparison> {
    first.validate().context("first trace")?;
    second.validate().context("second trace")?;
    ensure!(
        first.backend.backend_id != second.backend.backend_id,
        "comparison requires distinct backends"
    );
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = PerWheelSkidComparison {
        kind: PER_WHEEL_SKID_COMPARISON_KIND.to_string(),
        schema_version: PER_WHEEL_SKID_SCHEMA_VERSION,
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
pub(crate) struct WheelStation {
    pub(crate) entity: Entity,
    pub(crate) spec: WheelStationSpec,
}

pub(crate) fn spawn_wheel_stations(world: &mut World, chassis: Entity) -> [WheelStation; 4] {
    let names = ["front_left", "rear_left", "front_right", "rear_right"];
    let specs = wheel_station_specs();
    std::array::from_fn(|index| {
        let name = names[index];
        let spec = specs[index];
        let center_body_m = spec.center_body_m;
        let entity = spawn_named(world, name);
        world.entity_mut(entity).insert((
            RigidBody {
                mass_kg: 5.0,
                ..RigidBody::default()
            },
            frictionless_sphere(WHEEL_RADIUS_M),
            CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
            Transform3::from_translation_rotation(
                Vec3::new(center_body_m.x, 0.37 + center_body_m.y, center_body_m.z),
                Quat::IDENTITY,
            ),
            FixedJointDesc {
                parent: chassis,
                anchor_parent_m: center_body_m,
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
            },
            spec,
        ));
        WheelStation { entity, spec }
    })
}

pub(crate) fn wheel_station_specs() -> [WheelStationSpec; 4] {
    [
        Vec3::new(0.35, -0.25, 0.30),
        Vec3::new(-0.35, -0.25, 0.30),
        Vec3::new(0.35, -0.25, -0.30),
        Vec3::new(-0.35, -0.25, -0.30),
    ]
    .map(|center_body_m| WheelStationSpec {
        center_body_m,
        ..WheelStationSpec::default()
    })
}

// Each parameter is an independent named SI-unit quantity; bundling into a config struct here would only relocate the arity, not reduce it.
#[allow(clippy::too_many_arguments)]
fn sample(
    step: u64,
    command_phase: f64,
    command_voltage_v: [f64; 2],
    transform: Transform3,
    integrated_yaw_rad: f64,
    body: RigidBody,
    wheel_normal_load_n: [f64; 4],
    wheel_raw_normal_load_n: [f64; 4],
    wheel_velocity_rad_s: [f64; 4],
    wheel_longitudinal_force_n: [f64; 4],
    wheel_lateral_force_n: [f64; 4],
    wheel_friction_utilization: [f64; 4],
) -> PerWheelSkidSample {
    let (wrapped_yaw_rad, _, _) = y_up_euler_rad(transform.rotation);
    PerWheelSkidSample {
        step,
        sim_time_ticks: SimTime::from_ticks(step * PER_WHEEL_SKID_FIXED_DELTA_TICKS).ticks(),
        command_phase,
        command_voltage_v,
        privileged_position_world_m: transform.translation.to_array().map(f64::from),
        privileged_integrated_yaw_rad: integrated_yaw_rad,
        privileged_wrapped_yaw_rad: wrapped_yaw_rad,
        privileged_yaw_rate_rad_s: body.angular_velocity_rad_s.y,
        wheel_normal_load_n,
        wheel_raw_normal_load_n,
        wheel_velocity_rad_s,
        wheel_longitudinal_force_n,
        wheel_lateral_force_n,
        wheel_friction_utilization,
    }
}

pub(crate) fn wheel_plant_spec() -> LongitudinalMobilityPlantSpec {
    let reference_load_n = VEHICLE_MASS_KG * 9.806_65 / 4.0;
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: VEHICLE_MASS_KG,
        driven_wheel_count: 1,
        normal_load_per_driven_wheel_n: reference_load_n,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.0,
        road_friction_scale: 1.0,
        motor: DcMotorSpec::default(),
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec {
            radius_m: WHEEL_RADIUS_M,
            width_m: 0.05,
            inertia_kg_m2: 0.02,
            ..WheelAssemblySpec::default()
        },
        tire: CombinedSlipTireSpec {
            reference_load_n,
            longitudinal_stiffness_n: 2_500.0,
            lateral_stiffness_n: 3_500.0,
            longitudinal_relaxation_length_m: 0.03,
            lateral_relaxation_length_m: 0.05,
            ..CombinedSlipTireSpec::default()
        },
        longitudinal_load_transfer: None,
    }
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
        passed: value >= minimum && value <= maximum,
    }
}

fn validate_metrics(metrics: &[MobilityBenchmarkMetric], passed: bool) -> Result<()> {
    ensure!(!metrics.is_empty(), "metrics omitted");
    ensure!(
        metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
        "metrics are not strictly sorted"
    );
    for metric in metrics {
        ensure!(
            metric.value.is_finite()
                && metric.minimum.is_finite()
                && metric.maximum.is_finite()
                && metric.minimum <= metric.maximum,
            "metric {} is invalid",
            metric.id
        );
        ensure!(
            metric.passed == (metric.value >= metric.minimum && metric.value <= metric.maximum),
            "metric {} verdict mismatch",
            metric.id
        );
    }
    ensure!(
        passed == metrics.iter().all(|metric| metric.passed),
        "aggregate verdict mismatch"
    );
    Ok(())
}

fn comparison_metrics(
    first: &PerWheelSkidTrace,
    second: &PerWheelSkidTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    let mut metrics = vec![
        gap_metric(
            "final_integrated_yaw_gap_rad",
            "rad",
            first,
            second,
            "final_absolute_integrated_yaw_rad",
            0.5,
        )?,
        gap_metric(
            "final_yaw_rate_gap_rad_s",
            "rad/s",
            first,
            second,
            "final_absolute_yaw_rate_rad_s",
            0.5,
        )?,
        gap_metric(
            "horizontal_displacement_gap_m",
            "m",
            first,
            second,
            "final_horizontal_displacement_m",
            0.1,
        )?,
        gap_metric(
            "maximum_scrub_force_gap_n",
            "N",
            first,
            second,
            "maximum_total_lateral_scrub_force_n",
            100.0,
        )?,
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    Ok(metrics)
}

fn gap_metric(
    id: &str,
    unit: &str,
    first: &PerWheelSkidTrace,
    second: &PerWheelSkidTrace,
    source: &str,
    maximum: f64,
) -> Result<MobilityBenchmarkMetric> {
    let first_value = metric_value(first, source)?;
    let second_value = metric_value(second, source)?;
    Ok(metric(
        id,
        unit,
        (first_value - second_value).abs(),
        0.0,
        maximum,
    ))
}

fn metric_value(trace: &PerWheelSkidTrace, id: &str) -> Result<f64> {
    trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .map(|metric| metric.value)
        .with_context(|| format!("missing metric {id}"))
}

fn trace_digest(trace: &PerWheelSkidTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn comparison_digest(comparison: &PerWheelSkidComparison) -> Result<String> {
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
    fn rapier_per_wheel_skid_trace_is_passing_and_deterministic() {
        let first =
            run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let second =
            run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert!(first.passed, "{:#?}", first.metrics);
        assert_eq!(first, second);
        first.validate().unwrap();
        assert_eq!(
            serde_json::to_vec(&first).unwrap(),
            serde_json::to_vec(&second).unwrap()
        );
    }

    #[test]
    fn per_wheel_trace_rejects_raw_contact_evidence_tampering() {
        let mut trace =
            run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace.samples[1].wheel_raw_normal_load_n[0] += 1.0;
        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_per_wheel_skid_traces_pass_shared_tolerances() {
        use rne_physics_mujoco::MuJoCoBackend;

        let rapier =
            run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_per_wheel_skid_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(PER_WHEEL_SKID_FIXED_DELTA_TICKS)).unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_per_wheel_skid_traces(rapier, mujoco).unwrap();
        assert!(
            comparison.passed,
            "rapier={:#?}\nmujoco={:#?}\ncomparison={:#?}",
            comparison.first.metrics, comparison.second.metrics, comparison.metrics,
        );
    }
}
