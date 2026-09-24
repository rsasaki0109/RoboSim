//! Cross-backend suspended Ackermann response to metric rigid-road excitation.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, CollisionGroups, ExternalBodyWrench, JointState, MultibodyLink,
    PhysicsBackend, PhysicsBackendManifest, PhysicsCapability, PhysicsWorldDesc, RigidBody,
    RigidBodyInertia, RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, evaluate_suspension_strut,
    rigid_road_patch_geometry, sample_rigid_road_profile, LongitudinalDrivePathInput,
    LongitudinalDrivePathState, LongitudinalMobilityPlantSpec, RigidRoadPatchSpec,
    RigidRoadProfileSpec, SuspensionStrutSpec, WheelStationSpec,
};
use rne_world::{Transform3, WorldRandom};
use serde::{Deserialize, Serialize};

use crate::ackermann_suspension::{
    frictionless_cuboid, spawn_stations, suspension_spec, wheel_plant_spec, wheel_station_specs,
    CHASSIS_MASS_KG, CONTACT_LOAD_FILTER_TIME_CONSTANT_S, INITIAL_SUSPENSION_POSITION_M,
    SELF_COLLISION_GROUP, WHEEL_RADIUS_M,
};
use crate::MobilityBenchmarkMetric;

/// Stable artifact kind for one backend road-excitation trace.
pub const ROAD_EXCITATION_TRACE_KIND: &str = "rne_mobility_road_excitation_trace";
/// Stable artifact kind for a two-backend road-excitation comparison.
pub const ROAD_EXCITATION_COMPARISON_KIND: &str = "rne_mobility_road_excitation_comparison";
/// Road-excitation artifact schema.
pub const ROAD_EXCITATION_SCHEMA_VERSION: u32 = 1;
/// Portable `TaskSpec` identity shared by Rapier and `MuJoCo`.
pub const ROAD_EXCITATION_TASK_ID: &str = "mobility_ackermann_road_excitation_v1";
/// One-millisecond physics and tire integration step, in simulation ticks.
pub const ROAD_EXCITATION_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SETTLE_STEPS: u64 = 1_500;
const TOTAL_STEPS: u64 = 14_000;
const TRACE_STRIDE_STEPS: u64 = 50;
const DRIVE_VOLTAGE_V: f64 = 12.0;
const ROAD_HALF_WIDTH_M: f64 = 3.0;
const ROAD_THICKNESS_M: f64 = 0.4;
const WORLD_SEED: u64 = 0;
const CONTACT_EVENT_CONFIRM_STEPS: u64 = 5;
const CURB_NORMAL_X_THRESHOLD: f64 = 0.20;

/// One sampled physical response with per-wheel contact evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadExcitationSample {
    /// Completed physics step.
    pub step: u64,
    /// Chassis forward position in meters.
    pub chassis_x_m: f64,
    /// Chassis vertical position in meters.
    pub chassis_y_m: f64,
    /// Chassis vertical velocity in meters per second.
    pub chassis_vertical_velocity_m_s: f64,
    /// Finite-difference chassis vertical acceleration in meters per second squared.
    pub chassis_vertical_acceleration_m_s2: f64,
    /// Suspension coordinates in FL, RL, FR, RR order, in meters.
    pub suspension_position_m: [f64; 4],
    /// Conditioned normal loads in FL, RL, FR, RR order, in newtons.
    pub wheel_normal_load_n: [f64; 4],
    /// Whether each wheel has completed backend contact.
    pub wheel_in_contact: [bool; 4],
    /// Sampled canonical road-patch index per wheel, or `-1` in a profile gap.
    pub road_patch_index: [i32; 4],
}

impl RoadExcitationSample {
    fn validate(&self) -> Result<()> {
        ensure!(
            self.step > 0 && self.step <= TOTAL_STEPS,
            "sample step drift"
        );
        ensure!(
            [
                self.chassis_x_m,
                self.chassis_y_m,
                self.chassis_vertical_velocity_m_s,
                self.chassis_vertical_acceleration_m_s2,
            ]
            .into_iter()
            .chain(self.suspension_position_m)
            .chain(self.wheel_normal_load_n)
            .all(f64::is_finite),
            "non-finite road-excitation sample"
        );
        ensure!(
            self.wheel_normal_load_n.iter().all(|load| *load >= 0.0),
            "negative normal load"
        );
        ensure!(
            self.road_patch_index.iter().all(|index| {
                *index == -1
                    || (*index >= 0 && (*index as usize) < rigid_road_profile().patches.len())
            }),
            "road patch index drift"
        );
        Ok(())
    }
}

/// Self-verifying response of one backend to the exact road profile.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadExcitationTrace {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact task contract.
    pub task_spec: TaskSpec,
    /// Exact road geometry and friction profile.
    pub road_profile: RigidRoadProfileSpec,
    /// Exact motor, transmission, wheel, and tire profile.
    pub wheel_plant_spec: LongitudinalMobilityPlantSpec,
    /// Exact suspension profile.
    pub suspension_spec: SuspensionStrutSpec,
    /// Exact station geometry.
    pub wheel_station_specs: [WheelStationSpec; 4],
    /// Physics step in simulation ticks.
    pub fixed_delta_ticks: u64,
    /// Deterministic seed root.
    pub seed: u64,
    /// Completed steps.
    pub steps: u64,
    /// Ordered sampled physical response.
    pub samples: Vec<RoadExcitationSample>,
    /// SI-unit acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Aggregate verdict.
    pub passed: bool,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl RoadExcitationTrace {
    /// Recomputes frozen contracts, sample ordering, verdicts, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == ROAD_EXCITATION_TRACE_KIND, "trace kind drift");
        ensure!(
            self.schema_version == ROAD_EXCITATION_SCHEMA_VERSION,
            "schema drift"
        );
        self.backend.validate()?;
        self.task_spec.validate()?;
        ensure!(
            self.task_spec == road_excitation_task_spec(),
            "TaskSpec drift"
        );
        ensure!(
            self.road_profile == rigid_road_profile(),
            "road profile drift"
        );
        ensure!(self.road_profile.is_valid(), "invalid road profile");
        ensure!(self.wheel_plant_spec.is_valid(), "invalid wheel plant spec");
        ensure!(self.suspension_spec.is_valid(), "invalid suspension spec");
        ensure!(
            self.wheel_station_specs == wheel_station_specs(),
            "station drift"
        );
        ensure!(
            self.fixed_delta_ticks == ROAD_EXCITATION_FIXED_DELTA_TICKS
                && self.seed == WORLD_SEED
                && self.steps == TOTAL_STEPS,
            "runtime contract drift"
        );
        ensure!(!self.samples.is_empty(), "missing samples");
        ensure!(
            self.samples
                .windows(2)
                .all(|pair| pair[0].step < pair[1].step),
            "sample order drift"
        );
        for sample in &self.samples {
            sample.validate()?;
        }
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == trace_digest(self)?,
            "trace digest drift"
        );
        Ok(())
    }
}

/// Self-verifying SI-unit comparison between two backend traces.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadExcitationComparison {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// First complete backend trace.
    pub first: RoadExcitationTrace,
    /// Second complete backend trace.
    pub second: RoadExcitationTrace,
    /// Ordered cross-backend metric gaps.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Aggregate verdict.
    pub passed: bool,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl RoadExcitationComparison {
    /// Validates both subjects, shared contracts, gaps, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == ROAD_EXCITATION_COMPARISON_KIND,
            "comparison kind drift"
        );
        ensure!(
            self.schema_version == ROAD_EXCITATION_SCHEMA_VERSION,
            "comparison schema drift"
        );
        self.first.validate()?;
        self.second.validate()?;
        ensure!(
            self.first.task_spec == self.second.task_spec,
            "TaskSpec mismatch"
        );
        ensure!(
            self.first.road_profile == self.second.road_profile,
            "road profile mismatch"
        );
        ensure!(
            self.first.wheel_plant_spec == self.second.wheel_plant_spec,
            "wheel plant mismatch"
        );
        ensure!(
            self.first.suspension_spec == self.second.suspension_spec,
            "suspension mismatch"
        );
        ensure!(
            self.first.wheel_station_specs == self.second.wheel_station_specs,
            "wheel station mismatch"
        );
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "same backend"
        );
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == comparison_digest(self)?,
            "comparison digest drift"
        );
        Ok(())
    }
}

/// Frozen straight-road excitation task shared by both backends.
pub fn road_excitation_task_spec() -> TaskSpec {
    TaskSpec::new(
        ROAD_EXCITATION_TASK_ID,
        ROAD_EXCITATION_FIXED_DELTA_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new("diagnostic_chassis_pose", TensorDType::F64, vec![2], "m")
                .with_bounds(TensorBounds::broadcast(-20.0, 20.0)),
            TensorSpec::new(
                "diagnostic_vertical_kinematics",
                TensorDType::F64,
                vec![2],
                "m/s,m/s^2",
            )
            .with_bounds(TensorBounds::broadcast(-100.0, 100.0)),
            TensorSpec::new(
                "diagnostic_suspension_position",
                TensorDType::F64,
                vec![4],
                "m",
            )
            .with_bounds(TensorBounds::broadcast(-0.2, 0.2)),
            TensorSpec::new(
                "diagnostic_wheel_normal_load",
                TensorDType::F64,
                vec![4],
                "N",
            )
            .with_bounds(TensorBounds::broadcast(0.0, 20_000.0)),
            TensorSpec::new(
                "diagnostic_wheel_contact",
                TensorDType::Bool,
                vec![4],
                "bool",
            ),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "motor_terminal_voltage",
            TensorDType::F64,
            vec![4],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(0.0, DRIVE_VOLTAGE_V))]),
        RewardSpec::weighted_sum(vec![RewardTermSpec::new(
            "diagnostic_road_holding",
            1.0,
            "1",
        )]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "diagnostic_out_of_bounds",
                TerminationKind::Failure,
            )],
            Some(TOTAL_STEPS / 10),
        ),
        ResetSpec::splitmix64(false),
    )
}

/// Builds the canonical flat/grade/roughness/curb/drop rigid-road profile.
pub fn rigid_road_profile() -> RigidRoadProfileSpec {
    let mut patches = Vec::new();
    let mut start = Vec3::new(-3.0, 0.0, 0.0);
    append_patch(&mut patches, &mut start, 6.0, 0.0, 1.0);
    append_patch(&mut patches, &mut start, 1.0, 0.08, 1.0);
    append_patch(&mut patches, &mut start, 0.8, 0.0, 0.9);
    for grade_rad in [0.06, -0.06, 0.06, -0.06, 0.06, -0.06, 0.06, -0.06] {
        append_patch(&mut patches, &mut start, 0.25, grade_rad, 0.8);
    }
    start.y += 0.04;
    append_patch(&mut patches, &mut start, 0.8, 0.0, 0.85);
    start.y -= 0.04;
    append_patch(&mut patches, &mut start, 4.0, 0.0, 1.0);
    RigidRoadProfileSpec { patches }
}

fn append_patch(
    patches: &mut Vec<RigidRoadPatchSpec>,
    start: &mut Vec3,
    length_m: f64,
    grade_rad: f64,
    friction_scale: f64,
) {
    let tangent = Vec3::new(grade_rad.cos(), grade_rad.sin(), 0.0);
    patches.push(RigidRoadPatchSpec {
        surface_center_world_m: *start + tangent * (0.5 * length_m),
        surface_length_m: length_m,
        half_width_m: ROAD_HALF_WIDTH_M,
        thickness_m: ROAD_THICKNESS_M,
        grade_rad,
        friction_scale,
    });
    *start += tangent * length_m;
}

/// Runs the metric road profile through one rigid-body backend.
pub fn run_road_excitation_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<RoadExcitationTrace> {
    run_road_excitation_trace_with_suspension(backend, manifest, suspension_spec())
}

/// Runs the metric road profile with an explicitly supplied portable suspension.
pub fn run_road_excitation_trace_with_suspension<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    suspension: SuspensionStrutSpec,
) -> Result<RoadExcitationTrace> {
    run_road_excitation_trace_with_specs(backend, manifest, suspension, wheel_plant_spec())
}

/// Runs the metric road profile with explicit portable suspension and wheel-plant specs.
///
/// This parameterized entry point applies identified suspension and tire values to
/// the exact same executed task that [`run_road_excitation_trace`] runs with the
/// baseline fixture. [`RoadExcitationTrace::validate`] only checks that the retained
/// specs are individually valid; the wrapper evidence that supplies them is
/// responsible for binding each spec to its identification chain.
#[allow(clippy::too_many_lines)] // TODO(cleanup): split (341/150 lines); see PR body
pub fn run_road_excitation_trace_with_specs<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
    suspension: SuspensionStrutSpec,
    wheel_plant: LongitudinalMobilityPlantSpec,
) -> Result<RoadExcitationTrace> {
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
    let task_spec = road_excitation_task_spec();
    task_spec.validate()?;
    let road_profile = rigid_road_profile();
    ensure!(road_profile.is_valid(), "invalid road profile fixture");
    let plant = wheel_plant;
    ensure!(suspension.is_valid(), "invalid suspension spec");
    ensure!(plant.is_valid(), "invalid wheel plant spec");
    let fixed_delta = SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 48,
    })?;
    let mut world = World::new();
    world.insert_resource(WorldRandom::new(WORLD_SEED));
    for (index, patch) in road_profile.patches.iter().copied().enumerate() {
        let geometry = rigid_road_patch_geometry(patch)?;
        let road = spawn_named(&mut world, format!("road_patch_{index:03}"));
        world.entity_mut(road).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            frictionless_cuboid(geometry.solid_half_extents_m),
            geometry.solid_transform,
        ));
    }
    let chassis = spawn_named(&mut world, "road_excitation_chassis");
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

    let initial_x_m = world
        .get::<Transform3>(chassis)
        .context("initial chassis")?
        .translation
        .x;
    let mut drive_states = [LongitudinalDrivePathState::default(); 4];
    let mut conditioned_load_n = [0.0; 4];
    let mut pending_wrenches: Vec<ExternalBodyWrench> = Vec::new();
    let mut stable_contact = [false; 4];
    let mut contact_streak = [0_u64; 4];
    let mut no_contact_streak = [0_u64; 4];
    let mut contact_initialized = false;
    let mut lift_events = 0_u64;
    let mut recontact_events = 0_u64;
    let mut curb_face_contact_steps = 0_u64;
    let mut curb_face_normal_impulse_n_s = 0.0_f64;
    let mut maximum_curb_face_load_n = 0.0_f64;
    let mut maximum_grade_normal_x = 0.0_f64;
    let mut maximum_suspension_velocity_m_s = 0.0_f64;
    let mut maximum_vertical_acceleration_m_s2 = 0.0_f64;
    let mut squared_vertical_acceleration = 0.0_f64;
    let mut acceleration_samples = 0_u64;
    let mut previous_vertical_velocity_m_s = 0.0;
    let mut samples = Vec::new();

    for zero_based_step in 0..TOTAL_STEPS {
        backend.sync_from_ecs(&mut world, physics_world)?;
        for wrench in pending_wrenches.drain(..) {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;
        let step = zero_based_step + 1;
        let transform = *world.get::<Transform3>(chassis).context("chassis pose")?;
        let body = *world.get::<RigidBody>(chassis).context("chassis body")?;
        let vertical_acceleration_m_s2 =
            (body.linear_velocity_m_s.y - previous_vertical_velocity_m_s) / dt_s;
        previous_vertical_velocity_m_s = body.linear_velocity_m_s.y;
        let contacts = backend.contact_points(physics_world)?;
        let load_alpha = dt_s / (CONTACT_LOAD_FILTER_TIME_CONSTANT_S + dt_s);
        let mut suspension_position_m = [0.0; 4];
        let mut wheel_in_contact = [false; 4];
        let mut road_patch_index = [-1; 4];

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
            maximum_suspension_velocity_m_s =
                maximum_suspension_velocity_m_s.max(velocity_m_s.abs());
            world
                .entity_mut(station.slider)
                .insert(evaluate_suspension_strut(
                    suspension,
                    position_m,
                    velocity_m_s,
                )?);

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
            wheel_in_contact[index] = raw_patch.is_some();
            if step >= SETTLE_STEPS && contact_initialized {
                if wheel_in_contact[index] {
                    contact_streak[index] += 1;
                    no_contact_streak[index] = 0;
                    if !stable_contact[index]
                        && contact_streak[index] == CONTACT_EVENT_CONFIRM_STEPS
                    {
                        stable_contact[index] = true;
                        recontact_events += 1;
                    }
                } else {
                    no_contact_streak[index] += 1;
                    contact_streak[index] = 0;
                    if stable_contact[index]
                        && no_contact_streak[index] == CONTACT_EVENT_CONFIRM_STEPS
                    {
                        stable_contact[index] = false;
                        lift_events += 1;
                    }
                }
            }
            let road_sample =
                sample_rigid_road_profile(&road_profile, wheel_transform.translation)?;
            if let Some(sample) = road_sample {
                road_patch_index[index] = i32::try_from(sample.patch_index)?;
            }
            if let Some(patch) = raw_patch {
                if patch.normal_road_to_wheel_world.x.abs() > CURB_NORMAL_X_THRESHOLD {
                    curb_face_contact_steps += 1;
                    curb_face_normal_impulse_n_s += patch.normal_load_n * dt_s;
                    maximum_curb_face_load_n = maximum_curb_face_load_n.max(patch.normal_load_n);
                } else if road_sample.is_some_and(|sample| {
                    road_profile.patches[sample.patch_index].grade_rad.abs() > 0.07
                }) {
                    maximum_grade_normal_x =
                        maximum_grade_normal_x.max(patch.normal_road_to_wheel_world.x.abs());
                }
            }
            let bounded_load_n = raw_patch
                .map_or(0.0, |patch| patch.normal_load_n)
                .min(plant.tire.reference_load_n * plant.tire.maximum_load_ratio);
            conditioned_load_n[index] += load_alpha * (bounded_load_n - conditioned_load_n[index]);
            let driving_patch = raw_patch.filter(|patch| patch.normal_road_to_wheel_world.y >= 0.5);
            let driving_patch = driving_patch.map(|mut patch| {
                patch.normal_load_n = conditioned_load_n[index];
                patch
            });
            let mut station_plant = plant;
            station_plant.road_friction_scale =
                road_sample.map_or(0.0, |sample| sample.friction_scale);
            let evaluation = evaluate_longitudinal_drive_path(
                station_plant,
                drive_states[index],
                LongitudinalDrivePathInput {
                    carrier_patch: driving_patch,
                    forward_world,
                    lateral_world,
                    command_voltage_v: if step > SETTLE_STEPS {
                        DRIVE_VOLTAGE_V
                    } else {
                        0.0
                    },
                },
                dt_s,
            )?;
            drive_states[index] = evaluation.state;
            if let Some(wrench) = evaluation.tire_wrench {
                pending_wrenches.push(wrench);
            }
        }
        if step == SETTLE_STEPS {
            contact_initialized = true;
            stable_contact = wheel_in_contact;
            contact_streak = [CONTACT_EVENT_CONFIRM_STEPS; 4];
            no_contact_streak = [0; 4];
        }
        if step > SETTLE_STEPS {
            maximum_vertical_acceleration_m_s2 =
                maximum_vertical_acceleration_m_s2.max(vertical_acceleration_m_s2.abs());
            squared_vertical_acceleration += vertical_acceleration_m_s2.powi(2);
            acceleration_samples += 1;
        }
        if step % TRACE_STRIDE_STEPS == 0 || step == TOTAL_STEPS {
            samples.push(RoadExcitationSample {
                step,
                chassis_x_m: transform.translation.x,
                chassis_y_m: transform.translation.y,
                chassis_vertical_velocity_m_s: body.linear_velocity_m_s.y,
                chassis_vertical_acceleration_m_s2: vertical_acceleration_m_s2,
                suspension_position_m,
                wheel_normal_load_n: conditioned_load_n,
                wheel_in_contact,
                road_patch_index,
            });
        }
    }
    ensure!(acceleration_samples > 0, "missing response samples");
    let final_x_m = world
        .get::<Transform3>(chassis)
        .context("final chassis")?
        .translation
        .x;
    let mut metrics = vec![
        metric(
            "curb_face_contact_steps",
            "steps",
            curb_face_contact_steps as f64,
            1.0,
            20_000.0,
        ),
        metric(
            "curb_face_normal_impulse_n_s",
            "N*s",
            curb_face_normal_impulse_n_s,
            0.01,
            2_000.0,
        ),
        metric(
            "forward_displacement_m",
            "m",
            final_x_m - initial_x_m,
            5.0,
            20.0,
        ),
        metric(
            "maximum_curb_face_load_n",
            "N",
            maximum_curb_face_load_n,
            10.0,
            30_000.0,
        ),
        metric(
            "maximum_grade_normal_x",
            "1",
            maximum_grade_normal_x,
            0.03,
            0.20,
        ),
        metric(
            "maximum_suspension_velocity_m_s",
            "m/s",
            maximum_suspension_velocity_m_s,
            0.02,
            10.0,
        ),
        metric(
            "maximum_vertical_acceleration_m_s2",
            "m/s^2",
            maximum_vertical_acceleration_m_s2,
            0.1,
            5_000.0,
        ),
        metric(
            "recontact_events",
            "events",
            recontact_events as f64,
            1.0,
            1_000.0,
        ),
        metric(
            "rms_vertical_acceleration_m_s2",
            "m/s^2",
            (squared_vertical_acceleration / acceleration_samples as f64).sqrt(),
            0.01,
            100.0,
        ),
        metric(
            "wheel_lift_events",
            "events",
            lift_events as f64,
            1.0,
            1_000.0,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = RoadExcitationTrace {
        kind: ROAD_EXCITATION_TRACE_KIND.to_string(),
        schema_version: ROAD_EXCITATION_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        road_profile,
        wheel_plant_spec: plant,
        suspension_spec: suspension,
        wheel_station_specs: stations.map(|station| station.spec),
        fixed_delta_ticks: ROAD_EXCITATION_FIXED_DELTA_TICKS,
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
pub fn compare_road_excitation_traces(
    first: RoadExcitationTrace,
    second: RoadExcitationTrace,
) -> Result<RoadExcitationComparison> {
    first.validate()?;
    second.validate()?;
    let mut metrics = vec![
        gap_metric(&first, &second, "forward_displacement_m", "m", 0.5)?,
        // Peak constraint forces are solver- and stabilization-specific. Integrated
        // normal impulse is the portable rigid-impact quantity to compare.
        gap_metric(&first, &second, "curb_face_normal_impulse_n_s", "N*s", 50.0)?,
        gap_metric(
            &first,
            &second,
            "maximum_suspension_velocity_m_s",
            "m/s",
            1.0,
        )?,
        gap_metric(
            &first,
            &second,
            "rms_vertical_acceleration_m_s2",
            "m/s^2",
            10.0,
        )?,
        gap_metric(&first, &second, "wheel_lift_events", "events", 20.0)?,
        gap_metric(&first, &second, "recontact_events", "events", 20.0)?,
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut comparison = RoadExcitationComparison {
        kind: ROAD_EXCITATION_COMPARISON_KIND.to_string(),
        schema_version: ROAD_EXCITATION_SCHEMA_VERSION,
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

fn gap_metric(
    first: &RoadExcitationTrace,
    second: &RoadExcitationTrace,
    source: &str,
    unit: &str,
    maximum: f64,
) -> Result<MobilityBenchmarkMetric> {
    let gap = (metric_value(first, source)? - metric_value(second, source)?).abs();
    Ok(metric(&format!("{source}_gap"), unit, gap, 0.0, maximum))
}

fn metric_value(trace: &RoadExcitationTrace, id: &str) -> Result<f64> {
    trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .map(|metric| metric.value)
        .with_context(|| format!("missing metric {id}"))
}

fn validate_metrics(metrics: &[MobilityBenchmarkMetric], passed: bool) -> Result<()> {
    ensure!(!metrics.is_empty(), "missing metrics");
    ensure!(
        metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
        "metric order drift"
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

fn trace_digest(trace: &RoadExcitationTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn comparison_digest(comparison: &RoadExcitationComparison) -> Result<String> {
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
    fn road_profile_is_canonical_and_contains_grade_roughness_and_curb() {
        let profile = rigid_road_profile();
        assert!(profile.is_valid());
        assert!(profile.patches.iter().any(|patch| patch.grade_rad > 0.07));
        assert!(profile.patches.iter().any(|patch| patch.grade_rad < 0.0));
        assert!(profile.patches.windows(2).any(|pair| {
            let first_tangent = Vec3::new(pair[0].grade_rad.cos(), pair[0].grade_rad.sin(), 0.0);
            let second_tangent = Vec3::new(pair[1].grade_rad.cos(), pair[1].grade_rad.sin(), 0.0);
            let first_end =
                pair[0].surface_center_world_m + first_tangent * (0.5 * pair[0].surface_length_m);
            let second_start =
                pair[1].surface_center_world_m - second_tangent * (0.5 * pair[1].surface_length_m);
            (second_start.y - first_end.y).abs() >= 0.039
        }));
    }

    #[test]
    fn rapier_road_excitation_is_passing_and_deterministic() {
        let first =
            run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert!(first.passed, "{:#?}", first.metrics);
        first.validate().unwrap();
        let second =
            run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn trace_tampering_is_detected() {
        let mut trace =
            run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace.samples[1].wheel_normal_load_n[0] += 1.0;
        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_road_excitation_passes_shared_tolerances() {
        use rne_physics_mujoco::MuJoCoBackend;

        let rapier =
            run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_road_excitation_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS)).unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_road_excitation_traces(rapier, mujoco).unwrap();
        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();
    }
}
