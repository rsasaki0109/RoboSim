//! Backend-neutral physics conformance vectors and deterministic reports.

#![deny(missing_docs)]

use anyhow::{anyhow, Context};
#[cfg(feature = "mujoco")]
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation,
};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    capture_physics_snapshot, Collider, JointEffortMeasurement, JointState, MultibodyLink,
    PhysicsBackend, PhysicsBackendManifest, PhysicsCapability, PhysicsMaterial, PhysicsSnapshot,
    PhysicsWorldDesc, RaycastQuery, RevoluteJointDesc, RigidBody, RigidBodyType,
    PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION, PHYSICS_TOLERANCE_REGISTRY_VERSION,
};
use rne_physics_analytic::AnalyticBackend;
#[cfg(feature = "mujoco")]
use rne_physics_mujoco::MuJoCoBackend;
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Stable kind identifier for aggregate physics conformance reports.
pub const CONFORMANCE_REPORT_KIND: &str = "rne_physics_conformance_report";

/// Version of the shared capability-to-vector catalog.
pub const CONFORMANCE_CATALOG_VERSION: u16 = 4;

const BACKEND_ANALYTIC: &str = "analytic";
#[cfg(feature = "mujoco")]
const BACKEND_MUJOCO: &str = "mujoco";
const BACKEND_RAPIER: &str = "rapier";
const BACKEND_COMPARISON: &str = "analytic_vs_rapier";
#[cfg(feature = "mujoco")]
const BACKEND_ANALYTIC_MUJOCO: &str = "analytic_vs_mujoco";
#[cfg(feature = "mujoco")]
const BACKEND_RAPIER_MUJOCO: &str = "rapier_vs_mujoco";
const CASE_ANALYTIC_RIGID: &str = "analytic.rigid_body.free_fall";
#[cfg(feature = "mujoco")]
const CASE_MUJOCO_RIGID: &str = "mujoco.rigid_body.free_fall";
const CASE_RAPIER_RIGID: &str = "rapier.rigid_body.free_fall";
const CASE_RAPIER_ARTICULATION: &str = "rapier.articulation.revolute_limit";
const CASE_RAPIER_CONTACT: &str = "rapier.contact_force.resting_impulse";
#[cfg(feature = "mujoco")]
const CASE_MUJOCO_CONTACT: &str = "mujoco.contact_force.resting_impulse";
const CASE_RAPIER_RAYCAST: &str = "rapier.raycast_batch.ordered_hits";
const CASE_BACKEND_COMPARISON: &str = "analytic_vs_rapier.free_fall";
#[cfg(feature = "mujoco")]
const CASE_ANALYTIC_MUJOCO_COMPARISON: &str = "analytic_vs_mujoco.free_fall";
#[cfg(feature = "mujoco")]
const CASE_RAPIER_MUJOCO_COMPARISON: &str = "rapier_vs_mujoco.free_fall";
#[cfg(feature = "mujoco")]
const CASE_RAPIER_MUJOCO_DIAGNOSTIC: &str = "rapier_vs_mujoco.free_fall.diagnostic_perturbation";
#[cfg(feature = "mujoco")]
const DIAGNOSTIC_CONTRACT: &str = "rapier_vs_mujoco.position_delta_m.diagnostic";

const STEP_HZ: f64 = 60.0;
const FREE_FALL_STEPS: u64 = 60;
const GRAVITY_M_S2: f64 = -9.81;
const FREE_FALL_INITIAL_Y_M: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum MetricUnit {
    Metre,
    MetrePerSecond,
    Radian,
    NewtonSecond,
    NewtonMetre,
    Unitless,
}

impl MetricUnit {
    const fn symbol(self) -> &'static str {
        match self {
            Self::Metre => "m",
            Self::MetrePerSecond => "m/s",
            Self::Radian => "rad",
            Self::NewtonSecond => "N*s",
            Self::NewtonMetre => "N*m",
            Self::Unitless => "1",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ToleranceSpec {
    id: &'static str,
    case_id: &'static str,
    metric_id: &'static str,
    unit: MetricUnit,
    absolute: f64,
    relative: f64,
    rationale: &'static str,
}

impl ToleranceSpec {
    fn allowed_error(self, expected: f64) -> f64 {
        self.absolute.max(self.relative * expected.abs())
    }

    fn accepts(self, measured: f64, expected: f64) -> bool {
        measured.is_finite()
            && expected.is_finite()
            && (measured - expected).abs() <= self.allowed_error(expected)
    }
}

const TOLERANCES: &[ToleranceSpec] = &[
    ToleranceSpec {
        id: "analytic_free_fall_position_m_v1",
        case_id: CASE_ANALYTIC_RIGID,
        metric_id: "position_y_m",
        unit: MetricUnit::Metre,
        absolute: 1e-10,
        relative: 0.0,
        rationale: "f64 semi-implicit Euler vector has a closed discrete reference",
    },
    ToleranceSpec {
        id: "rapier_free_fall_position_m_v1",
        case_id: CASE_RAPIER_RIGID,
        metric_id: "position_y_m",
        unit: MetricUnit::Metre,
        absolute: 0.03,
        relative: 0.0,
        rationale: "Rapier f32 integration is within 3 cm of continuous free fall at 60 Hz",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "mujoco_free_fall_position_m_v1",
        case_id: CASE_MUJOCO_RIGID,
        metric_id: "position_y_m",
        unit: MetricUnit::Metre,
        absolute: 1e-9,
        relative: 0.0,
        rationale: "MuJoCo f64 Euler integration follows the closed discrete reference",
    },
    ToleranceSpec {
        id: "free_fall_velocity_m_s_v1",
        case_id: "shared.free_fall",
        metric_id: "velocity_y_m_s",
        unit: MetricUnit::MetrePerSecond,
        absolute: 0.001,
        relative: 0.0,
        rationale: "one second of constant gravity permits only f32 pipeline rounding",
    },
    ToleranceSpec {
        id: "kinematic_translation_m_v1",
        case_id: "shared.kinematic_pose",
        metric_id: "translation_error_m",
        unit: MetricUnit::Metre,
        absolute: 1e-6,
        relative: 0.0,
        rationale: "externally supplied kinematic translation permits only f32 conversion rounding",
    },
    ToleranceSpec {
        id: "kinematic_rotation_rad_v1",
        case_id: "shared.kinematic_pose",
        metric_id: "rotation_error_rad",
        unit: MetricUnit::Radian,
        absolute: 1e-6,
        relative: 0.0,
        rationale:
            "externally supplied kinematic rotation permits only normalized f32 conversion rounding",
    },
    ToleranceSpec {
        id: "revolute_anchor_m_v1",
        case_id: CASE_RAPIER_ARTICULATION,
        metric_id: "anchor_error_m",
        unit: MetricUnit::Metre,
        absolute: 0.01,
        relative: 0.0,
        rationale: "iterative constraint stabilization may leave millimetre anchor error",
    },
    ToleranceSpec {
        id: "revolute_limit_rad_v1",
        case_id: CASE_RAPIER_ARTICULATION,
        metric_id: "joint_angle_abs_rad",
        unit: MetricUnit::Radian,
        absolute: 0.03,
        relative: 0.0,
        rationale: "motor load and discrete limit stabilization permit 0.03 rad slack",
    },
    ToleranceSpec {
        id: "resting_impulse_n_s_v1",
        case_id: CASE_RAPIER_CONTACT,
        metric_id: "mean_normal_impulse_n_s",
        unit: MetricUnit::NewtonSecond,
        absolute: 0.015,
        relative: 0.35,
        rationale:
            "TGS contact stabilization biases manifold impulse above mass times gravity times dt",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "mujoco_resting_impulse_n_s_v1",
        case_id: CASE_MUJOCO_CONTACT,
        metric_id: "mean_normal_impulse_n_s",
        unit: MetricUnit::NewtonSecond,
        absolute: 0.002,
        relative: 0.02,
        rationale: "settled MuJoCo contact force integrated over one fixed step tracks body weight",
    },
    ToleranceSpec {
        id: "direct_revolute_effort_nm_v1",
        case_id: "shared.joint_effort_measurement",
        metric_id: "measured_effort_nm",
        unit: MetricUnit::NewtonMetre,
        absolute: 1e-6,
        relative: 1e-6,
        rationale: "direct native actuator effort retains the commanded SI value within backend numeric conversion rounding",
    },
    ToleranceSpec {
        id: "raycast_distance_m_v1",
        case_id: CASE_RAPIER_RAYCAST,
        metric_id: "hit_distance_m",
        unit: MetricUnit::Metre,
        absolute: 1e-5,
        relative: 0.0,
        rationale: "axis-aligned cuboid intersections are exact up to f32 conversion",
    },
    ToleranceSpec {
        id: "backend_free_fall_position_delta_m_v1",
        case_id: CASE_BACKEND_COMPARISON,
        metric_id: "position_delta_m",
        unit: MetricUnit::Metre,
        absolute: 0.10,
        relative: 0.0,
        rationale:
            "analytic semi-implicit and Rapier integration differ by one O(dt) position term",
    },
    ToleranceSpec {
        id: "backend_free_fall_velocity_delta_m_s_v1",
        case_id: CASE_BACKEND_COMPARISON,
        metric_id: "velocity_delta_m_s",
        unit: MetricUnit::MetrePerSecond,
        absolute: 0.001,
        relative: 0.0,
        rationale: "both solvers integrate the same constant acceleration for the velocity state",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "analytic_mujoco_free_fall_position_delta_m_v1",
        case_id: CASE_ANALYTIC_MUJOCO_COMPARISON,
        metric_id: "position_delta_m",
        unit: MetricUnit::Metre,
        absolute: 1e-9,
        relative: 0.0,
        rationale: "both f64 semi-implicit integrations follow the same fixed-step reference",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "analytic_mujoco_free_fall_velocity_delta_m_s_v1",
        case_id: CASE_ANALYTIC_MUJOCO_COMPARISON,
        metric_id: "velocity_delta_m_s",
        unit: MetricUnit::MetrePerSecond,
        absolute: 1e-9,
        relative: 0.0,
        rationale: "both backends integrate identical constant acceleration in f64",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "rapier_mujoco_free_fall_position_delta_m_v1",
        case_id: CASE_RAPIER_MUJOCO_COMPARISON,
        metric_id: "position_delta_m",
        unit: MetricUnit::Metre,
        absolute: 0.10,
        relative: 0.0,
        rationale: "Rapier and MuJoCo integration conventions differ by one bounded O(dt) term",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "rapier_mujoco_free_fall_velocity_delta_m_s_v1",
        case_id: CASE_RAPIER_MUJOCO_COMPARISON,
        metric_id: "velocity_delta_m_s",
        unit: MetricUnit::MetrePerSecond,
        absolute: 0.001,
        relative: 0.0,
        rationale: "both solvers integrate the same constant acceleration for velocity",
    },
    #[cfg(feature = "mujoco")]
    ToleranceSpec {
        id: "diagnostic_position_delta_m_v1",
        case_id: CASE_RAPIER_MUJOCO_DIAGNOSTIC,
        metric_id: "position_delta_m",
        unit: MetricUnit::Metre,
        absolute: 0.01,
        relative: 0.0,
        rationale:
            "deliberately strict fault-injection bound used only to prove divergence capture",
    },
];

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ToleranceReport {
    id: String,
    absolute: f64,
    relative: f64,
    allowed_error: f64,
    rationale: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct MetricReport {
    id: String,
    unit: String,
    measured: f64,
    expected: f64,
    absolute_error: f64,
    tolerance: ToleranceReport,
    passed: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CaseReport {
    id: String,
    backend: String,
    capability: PhysicsCapability,
    passed: bool,
    snapshot_hash: Option<u64>,
    metrics: Vec<MetricReport>,
    detail: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct BackendReport {
    manifest: PhysicsBackendManifest,
    runtime_capabilities: Vec<PhysicsCapability>,
    manifest_passed: bool,
    covered_capabilities: Vec<PhysicsCapability>,
    coverage_passed: bool,
    detail: String,
}

/// Deterministic result of the complete backend capability catalog.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConformanceReport {
    kind: String,
    schema_version: u16,
    catalog_version: u16,
    tolerance_registry_version: u16,
    backends: Vec<BackendReport>,
    cases: Vec<CaseReport>,
    all_passed: bool,
}

impl ConformanceReport {
    /// Returns true when every case and advertised-capability coverage check passed.
    pub const fn all_passed(&self) -> bool {
        self.all_passed
    }
}

/// Result of applying the shared capability catalog to one backend factory.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BackendConformance {
    backend: BackendReport,
    cases: Vec<CaseReport>,
    all_passed: bool,
}

impl BackendConformance {
    /// Returns true when the manifest matches runtime declarations and all cases pass.
    pub const fn all_passed(&self) -> bool {
        self.all_passed
    }
}

#[derive(Clone, Debug)]
struct FreeFallResult {
    position_y_m: f64,
    velocity_y_m_s: f64,
    snapshot: PhysicsSnapshot,
}

struct BackendExecution {
    conformance: BackendConformance,
    free_fall: anyhow::Result<FreeFallResult>,
}

fn tolerance(id: &str) -> &'static ToleranceSpec {
    TOLERANCES
        .iter()
        .find(|tolerance| tolerance.id == id)
        .unwrap_or_else(|| panic!("unknown conformance tolerance {id}"))
}

/// Executes every deterministic and tolerance-bounded physics conformance case.
pub fn run_conformance() -> ConformanceReport {
    let analytic = execute_backend(AnalyticBackend::manifest(), &AnalyticBackend::new);
    let rapier = execute_backend(RapierBackend::manifest(), &RapierBackend::new);
    let mut cases = analytic.conformance.cases.clone();
    cases.extend(rapier.conformance.cases.clone());
    cases.push(comparison_case(
        analytic.free_fall.as_ref(),
        rapier.free_fall.as_ref(),
        CASE_BACKEND_COMPARISON,
        BACKEND_COMPARISON,
        "backend_free_fall_position_delta_m_v1",
        "backend_free_fall_velocity_delta_m_s_v1",
    ));
    let backends = vec![analytic.conformance.backend, rapier.conformance.backend];
    #[cfg(feature = "mujoco")]
    let backends = {
        let mut backends = backends;
        let mujoco = execute_backend(MuJoCoBackend::manifest(), &mujoco_backend);
        cases.extend(mujoco.conformance.cases.clone());
        cases.push(comparison_case(
            analytic.free_fall.as_ref(),
            mujoco.free_fall.as_ref(),
            CASE_ANALYTIC_MUJOCO_COMPARISON,
            BACKEND_ANALYTIC_MUJOCO,
            "analytic_mujoco_free_fall_position_delta_m_v1",
            "analytic_mujoco_free_fall_velocity_delta_m_s_v1",
        ));
        cases.push(comparison_case(
            rapier.free_fall.as_ref(),
            mujoco.free_fall.as_ref(),
            CASE_RAPIER_MUJOCO_COMPARISON,
            BACKEND_RAPIER_MUJOCO,
            "rapier_mujoco_free_fall_position_delta_m_v1",
            "rapier_mujoco_free_fall_velocity_delta_m_s_v1",
        ));
        backends.push(mujoco.conformance.backend);
        backends
    };
    cases.sort_by(|left, right| left.id.cmp(&right.id));

    let all_passed = cases.iter().all(|case| case.passed)
        && backends
            .iter()
            .all(|backend| backend.manifest_passed && backend.coverage_passed);
    ConformanceReport {
        kind: CONFORMANCE_REPORT_KIND.to_string(),
        schema_version: PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION,
        catalog_version: CONFORMANCE_CATALOG_VERSION,
        tolerance_registry_version: PHYSICS_TOLERANCE_REGISTRY_VERSION,
        backends,
        cases,
        all_passed,
    }
}

#[cfg(feature = "mujoco")]
#[derive(Clone, Debug, PartialEq)]
struct FreeFallTraceSample {
    step: u64,
    sim_time_ticks: u64,
    position_y_m: f64,
    velocity_y_m_s: f64,
    snapshot_hash: u64,
}

/// Produces an intentionally failing Rapier-vs-MuJoCo diagnostic report and
/// its existing-schema Behavior replay.
///
/// The normal cross-backend contract remains unchanged and passing. This
/// diagnostic adds a clearly named 1 cm fault-injection bound, locates the
/// first fixed step that exceeds it, and records both backend observations up
/// to that violation for Failure Capsule packaging.
#[cfg(feature = "mujoco")]
pub fn run_divergence_diagnostic() -> anyhow::Result<(ConformanceReport, BehaviorReplayArtifact)> {
    let mut report = run_conformance();
    anyhow::ensure!(
        report.all_passed(),
        "baseline conformance must pass before injecting a diagnostic divergence"
    );

    let rapier = run_free_fall_trace(RapierBackend::new())?;
    let mujoco = run_free_fall_trace(mujoco_backend())?;
    anyhow::ensure!(
        rapier.len() == mujoco.len(),
        "backend traces have different lengths"
    );
    let tolerance = tolerance("diagnostic_position_delta_m_v1");
    let violation_index = rapier
        .iter()
        .zip(&mujoco)
        .position(|(rapier, mujoco)| {
            rapier.step > 0
                && !tolerance.accepts((rapier.position_y_m - mujoco.position_y_m).abs(), 0.0)
        })
        .context("fault-injection bound did not expose a cross-backend divergence")?;
    let rapier_violation = &rapier[violation_index];
    let mujoco_violation = &mujoco[violation_index];
    let position_delta_m = (rapier_violation.position_y_m - mujoco_violation.position_y_m).abs();
    let diagnostic_metrics = vec![metric(
        "position_delta_m",
        position_delta_m,
        0.0,
        tolerance.id,
    )];
    anyhow::ensure!(
        diagnostic_metrics.iter().any(|metric| !metric.passed),
        "diagnostic case unexpectedly passed"
    );
    report.cases.push(CaseReport {
        id: CASE_RAPIER_MUJOCO_DIAGNOSTIC.to_string(),
        backend: BACKEND_RAPIER_MUJOCO.to_string(),
        capability: PhysicsCapability::RigidBody,
        passed: false,
        snapshot_hash: None,
        metrics: diagnostic_metrics,
        detail: format!(
            "deliberate 1 cm fault-injection bound first exceeded at completed step {}; production 10 cm profile remains passing; Rapier follows its continuous-leaning integration while MuJoCo follows the semi-implicit reference",
            rapier_violation.step
        ),
    });
    report.cases.sort_by(|left, right| left.id.cmp(&right.id));
    report.all_passed = false;

    let descriptor = BehaviorContractDescriptor {
        name: DIAGNOSTIC_CONTRACT.to_string(),
        kind: BehaviorContractKind::Always,
        entities: vec!["free_fall_body".to_string()],
    };
    let frames = rapier
        .iter()
        .zip(&mujoco)
        .take(violation_index + 1)
        .map(|(rapier, mujoco)| BehaviorReplayFrame {
            step: rapier.step,
            sim_time_ticks: rapier.sim_time_ticks,
            action: if rapier.step == 0 {
                BehaviorReplayAction::InitialObservation
            } else {
                BehaviorReplayAction::Advance
            },
            observation: serde_json::json!({
                "case_id": CASE_RAPIER_MUJOCO_DIAGNOSTIC,
                "rapier": {
                    "position_y_m": rapier.position_y_m,
                    "velocity_y_m_s": rapier.velocity_y_m_s,
                    "snapshot_hash_hex": format!("{:016x}", rapier.snapshot_hash),
                },
                "mujoco": {
                    "position_y_m": mujoco.position_y_m,
                    "velocity_y_m_s": mujoco.velocity_y_m_s,
                    "snapshot_hash_hex": format!("{:016x}", mujoco.snapshot_hash),
                },
                "position_delta_m": (rapier.position_y_m - mujoco.position_y_m).abs(),
                "diagnostic_tolerance": {
                    "id": tolerance.id,
                    "absolute_m": tolerance.absolute,
                },
            }),
            state_digest: divergence_state_digest(rapier, mujoco),
        })
        .collect::<Vec<_>>();
    let final_frame = frames.last().context("diagnostic replay is empty")?;
    let violation = BehaviorViolation {
        step: final_frame.step,
        sim_time_ticks: final_frame.sim_time_ticks,
        state_digest: final_frame.state_digest,
        entities: descriptor.entities.clone(),
        message: format!(
            "Rapier-vs-MuJoCo free-fall position delta {position_delta_m:.12} m exceeded injected 0.010000000000 m bound"
        ),
    };
    let replay = BehaviorReplayArtifact::new(
        "physics_conformance_rapier_vs_mujoco_free_fall",
        fnv1a64(b"physics_conformance_rapier_vs_mujoco_free_fall_v1"),
        0,
        fixed_dt().ticks(),
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor,
            violation,
        },
    )?;
    Ok((report, replay))
}

#[cfg(feature = "mujoco")]
fn run_free_fall_trace<B: PhysicsBackend>(
    mut backend: B,
) -> anyhow::Result<Vec<FreeFallTraceSample>> {
    let mut world = World::new();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, GRAVITY_M_S2, 0.0),
        ..PhysicsWorldDesc::default()
    })?;
    let body = spawn_named(&mut world, "free_fall_body");
    world.entity_mut(body).insert((
        RigidBody {
            mass_kg: 2.0,
            linear_velocity_m_s: Vec3::new(1.0, 0.0, 0.0),
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        Transform3::from_translation_rotation(
            Vec3::new(0.0, FREE_FALL_INITIAL_Y_M, 0.0),
            Quat::IDENTITY,
        ),
    ));
    let dt = fixed_dt();
    let mut samples = Vec::with_capacity(FREE_FALL_STEPS as usize + 1);
    samples.push(free_fall_trace_sample(&world, body, &[], 0, 0)?);
    for step in 1..=FREE_FALL_STEPS {
        step_backend(&mut backend, &mut world, physics_world, dt)?;
        let contacts = if backend
            .capabilities()
            .contains(&PhysicsCapability::ContactForce)
        {
            backend.contacts(physics_world)?
        } else {
            &[]
        };
        samples.push(free_fall_trace_sample(
            &world,
            body,
            contacts,
            step,
            dt.ticks() * step,
        )?);
    }
    Ok(samples)
}

#[cfg(feature = "mujoco")]
fn free_fall_trace_sample(
    world: &World,
    body: Entity,
    contacts: &[rne_physics::ContactEvent],
    step: u64,
    sim_time_ticks: u64,
) -> anyhow::Result<FreeFallTraceSample> {
    let transform = world
        .get::<Transform3>(body)
        .context("free-fall trace transform missing")?;
    let rigid_body = world
        .get::<RigidBody>(body)
        .context("free-fall trace rigid body missing")?;
    let snapshot = capture_physics_snapshot(world, contacts, step, sim_time_ticks)?;
    Ok(FreeFallTraceSample {
        step,
        sim_time_ticks,
        position_y_m: transform.translation.y,
        velocity_y_m_s: rigid_body.linear_velocity_m_s.y,
        snapshot_hash: snapshot.stable_hash(),
    })
}

#[cfg(feature = "mujoco")]
fn divergence_state_digest(rapier: &FreeFallTraceSample, mujoco: &FreeFallTraceSample) -> u64 {
    let mut bytes = Vec::with_capacity(48);
    bytes.extend_from_slice(&rapier.snapshot_hash.to_le_bytes());
    bytes.extend_from_slice(&mujoco.snapshot_hash.to_le_bytes());
    bytes.extend_from_slice(&rapier.position_y_m.to_bits().to_le_bytes());
    bytes.extend_from_slice(&mujoco.position_y_m.to_bits().to_le_bytes());
    bytes.extend_from_slice(&rapier.velocity_y_m_s.to_bits().to_le_bytes());
    bytes.extend_from_slice(&mujoco.velocity_y_m_s.to_bits().to_le_bytes());
    fnv1a64(&bytes)
}

#[cfg(feature = "mujoco")]
fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

/// Applies the shared v2 capability catalog to an arbitrary backend factory.
///
/// A backend may be called through this API before it is accepted into the
/// built-in aggregate. Missing tolerance profiles or capability vectors become
/// deterministic failing cases instead of being silently skipped.
pub fn run_backend_conformance<B, F>(
    manifest: PhysicsBackendManifest,
    factory: F,
) -> BackendConformance
where
    B: PhysicsBackend,
    F: Fn() -> B,
{
    execute_backend(manifest, &factory).conformance
}

fn execute_backend<B, F>(manifest: PhysicsBackendManifest, factory: &F) -> BackendExecution
where
    B: PhysicsBackend,
    F: Fn() -> B,
{
    let backend_id = if manifest.backend_id.trim().is_empty() {
        "invalid_backend"
    } else {
        manifest.backend_id.as_str()
    };
    let runtime_capabilities = factory().capabilities().to_vec();
    let free_fall = if manifest
        .capabilities
        .contains(&PhysicsCapability::RigidBody)
    {
        run_free_fall(factory())
    } else {
        Err(anyhow!("backend does not advertise rigid_body"))
    };
    let mut cases = manifest
        .capabilities
        .iter()
        .copied()
        .map(|capability| match capability {
            PhysicsCapability::RigidBody => {
                let id = capability_case_id(backend_id, capability);
                match free_fall_position_contract(backend_id) {
                    Some((expected_y_m, tolerance_id)) => result_case(
                        &id,
                        backend_id,
                        capability,
                        free_fall.as_ref().map(|result| {
                            free_fall_case(
                                &id,
                                backend_id,
                                capability,
                                result,
                                expected_y_m,
                                tolerance_id,
                            )
                        }),
                    ),
                    None => failed_case(
                        &id,
                        backend_id,
                        capability,
                        "no named free-fall tolerance profile is registered for this backend"
                            .to_string(),
                    ),
                }
            }
            PhysicsCapability::KinematicBody => result_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                capability,
                run_kinematic_case(factory(), backend_id),
            ),
            PhysicsCapability::DeterministicStep => determinism_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                factory,
            ),
            PhysicsCapability::Articulation => result_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                capability,
                run_articulation_case(factory(), backend_id),
            ),
            PhysicsCapability::ContactForce => result_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                capability,
                run_contact_case(factory(), backend_id),
            ),
            PhysicsCapability::RaycastBatch => result_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                capability,
                run_raycast_case(factory(), backend_id),
            ),
            PhysicsCapability::JointEffortMeasurement => result_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                capability,
                run_joint_effort_measurement_case(factory(), backend_id),
            ),
            PhysicsCapability::GpuRigidBody | PhysicsCapability::SoftBody => failed_case(
                &capability_case_id(backend_id, capability),
                backend_id,
                capability,
                format!(
                    "advertised capability has no shared conformance vector in catalog v{CONFORMANCE_CATALOG_VERSION}"
                ),
            ),
        })
        .collect::<Vec<_>>();
    cases.sort_by(|left, right| left.id.cmp(&right.id));
    let backend = backend_report(manifest, runtime_capabilities, &cases);
    let all_passed =
        backend.manifest_passed && backend.coverage_passed && cases.iter().all(|case| case.passed);
    BackendExecution {
        conformance: BackendConformance {
            backend,
            cases,
            all_passed,
        },
        free_fall,
    }
}

fn capability_case_id(backend: &str, capability: PhysicsCapability) -> String {
    let suffix = match capability {
        PhysicsCapability::RigidBody => "rigid_body.free_fall",
        PhysicsCapability::KinematicBody => "kinematic_body.external_pose",
        PhysicsCapability::Articulation => "articulation.revolute_limit",
        PhysicsCapability::GpuRigidBody => "gpu_rigid_body.catalog_missing",
        PhysicsCapability::DeterministicStep => "deterministic_step.repeat_snapshot",
        PhysicsCapability::SoftBody => "soft_body.catalog_missing",
        PhysicsCapability::ContactForce => "contact_force.resting_impulse",
        PhysicsCapability::RaycastBatch => "raycast_batch.ordered_hits",
        PhysicsCapability::JointEffortMeasurement => {
            "joint_effort_measurement.direct_revolute_effort"
        }
    };
    format!("{backend}.{suffix}")
}

fn free_fall_position_contract(backend: &str) -> Option<(f64, &'static str)> {
    match backend {
        BACKEND_ANALYTIC => Some((
            semi_implicit_free_fall_y(),
            "analytic_free_fall_position_m_v1",
        )),
        BACKEND_RAPIER => Some((continuous_free_fall_y(), "rapier_free_fall_position_m_v1")),
        #[cfg(feature = "mujoco")]
        BACKEND_MUJOCO => Some((
            semi_implicit_free_fall_y(),
            "mujoco_free_fall_position_m_v1",
        )),
        _ => None,
    }
}

fn run_free_fall<B: PhysicsBackend>(mut backend: B) -> anyhow::Result<FreeFallResult> {
    let mut world = World::new();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, GRAVITY_M_S2, 0.0),
        ..PhysicsWorldDesc::default()
    })?;
    let body = spawn_named(&mut world, "free_fall_body");
    world.entity_mut(body).insert((
        RigidBody {
            mass_kg: 2.0,
            linear_velocity_m_s: Vec3::new(1.0, 0.0, 0.0),
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        Transform3::from_translation_rotation(
            Vec3::new(0.0, FREE_FALL_INITIAL_Y_M, 0.0),
            Quat::IDENTITY,
        ),
    ));
    let dt = fixed_dt();
    for _ in 0..FREE_FALL_STEPS {
        backend.sync_from_ecs(&mut world, physics_world)?;
        backend.step(physics_world, dt)?;
        backend.sync_to_ecs(&mut world, physics_world)?;
    }
    let contacts = if backend
        .capabilities()
        .contains(&PhysicsCapability::ContactForce)
    {
        backend.contacts(physics_world)?.to_vec()
    } else {
        Vec::new()
    };
    let snapshot = capture_physics_snapshot(
        &world,
        &contacts,
        FREE_FALL_STEPS,
        dt.ticks() * FREE_FALL_STEPS,
    )?;
    let transform = world
        .get::<Transform3>(body)
        .context("free-fall transform missing")?;
    let rigid_body = world
        .get::<RigidBody>(body)
        .context("free-fall rigid body missing")?;
    Ok(FreeFallResult {
        position_y_m: transform.translation.y,
        velocity_y_m_s: rigid_body.linear_velocity_m_s.y,
        snapshot,
    })
}

fn free_fall_case(
    case_id: &str,
    backend: &str,
    capability: PhysicsCapability,
    result: &FreeFallResult,
    expected_y_m: f64,
    position_tolerance: &str,
) -> CaseReport {
    let metrics = vec![
        metric(
            "position_y_m",
            result.position_y_m,
            expected_y_m,
            position_tolerance,
        ),
        metric(
            "velocity_y_m_s",
            result.velocity_y_m_s,
            GRAVITY_M_S2,
            "free_fall_velocity_m_s_v1",
        ),
    ];
    CaseReport {
        id: case_id.to_string(),
        backend: backend.to_string(),
        capability,
        passed: metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(result.snapshot.stable_hash()),
        metrics,
        detail: "shared 60 Hz free-fall vector".to_string(),
    }
}

fn run_kinematic_case<B: PhysicsBackend>(
    mut backend: B,
    backend_id: &str,
) -> anyhow::Result<CaseReport> {
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::ZERO,
        ..PhysicsWorldDesc::default()
    })?;
    let mut world = World::new();
    let body = spawn_named(&mut world, "kinematic_body");
    world.entity_mut(body).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        Collider::sphere(0.1),
        Transform3::default(),
    ));
    let dt = fixed_dt();
    step_backend(&mut backend, &mut world, physics_world, dt)?;

    let target = Transform3::from_translation_rotation(
        Vec3::new(1.25, -0.5, 2.75),
        Quat::from_rotation_z(0.3),
    );
    *world
        .get_mut::<Transform3>(body)
        .context("kinematic transform missing before external pose")? = target;
    step_backend(&mut backend, &mut world, physics_world, dt)?;
    let actual = *world
        .get::<Transform3>(body)
        .context("kinematic transform missing after step")?;
    let translation_error_m = actual.translation.distance(target.translation);
    let rotation_dot = actual.rotation.dot(target.rotation).abs().clamp(0.0, 1.0);
    let rotation_error_rad = 2.0 * rotation_dot.acos();
    let metrics = vec![
        metric(
            "translation_error_m",
            translation_error_m,
            0.0,
            "kinematic_translation_m_v1",
        ),
        metric(
            "rotation_error_rad",
            rotation_error_rad,
            0.0,
            "kinematic_rotation_rad_v1",
        ),
    ];
    let contacts = if backend
        .capabilities()
        .contains(&PhysicsCapability::ContactForce)
    {
        backend.contacts(physics_world)?.to_vec()
    } else {
        Vec::new()
    };
    let snapshot = capture_physics_snapshot(&world, &contacts, 2, dt.ticks() * 2)?;
    Ok(CaseReport {
        id: capability_case_id(backend_id, PhysicsCapability::KinematicBody),
        backend: backend_id.to_string(),
        capability: PhysicsCapability::KinematicBody,
        passed: metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(snapshot.stable_hash()),
        metrics,
        detail: "externally supplied pose remains authoritative across a fixed step".to_string(),
    })
}

fn determinism_case<B: PhysicsBackend>(
    case_id: &str,
    backend: &str,
    factory: impl Fn() -> B,
) -> CaseReport {
    let runs = (run_free_fall(factory()), run_free_fall(factory()));
    match runs {
        (Ok(first), Ok(second)) => {
            let passed = first.snapshot == second.snapshot
                && first.snapshot.stable_hash() == second.snapshot.stable_hash();
            CaseReport {
                id: case_id.to_string(),
                backend: backend.to_string(),
                capability: PhysicsCapability::DeterministicStep,
                passed,
                snapshot_hash: Some(first.snapshot.stable_hash()),
                metrics: Vec::new(),
                detail: if passed {
                    "two fresh executions produced identical canonical snapshots".to_string()
                } else {
                    "repeat executions diverged".to_string()
                },
            }
        }
        (Err(error), _) | (_, Err(error)) => failed_case(
            case_id,
            backend,
            PhysicsCapability::DeterministicStep,
            error.to_string(),
        ),
    }
}

fn run_articulation_case<B: PhysicsBackend>(
    mut backend: B,
    backend_id: &str,
) -> anyhow::Result<CaseReport> {
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::ZERO,
        solver_iterations: 16,
    })?;
    let mut world = World::new();
    let parent = spawn_named(&mut world, "joint_parent");
    world.entity_mut(parent).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        MultibodyLink,
        Transform3::default(),
    ));
    let child = spawn_named(&mut world, "joint_child");
    let anchor_child_m = Vec3::new(0.0, 1.0, 0.0);
    world.entity_mut(child).insert((
        RigidBody::default(),
        Collider::sphere(0.05),
        MultibodyLink,
        Transform3::from_translation_rotation(Vec3::new(0.0, -1.0, 0.0), Quat::IDENTITY),
        RevoluteJointDesc {
            parent,
            axis: Vec3::Z,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m,
            lower_rad: Some(-0.2),
            upper_rad: Some(0.2),
        },
        rne_physics::JointActuation::RevoluteVelocity {
            target_velocity_rad_s: 2.0,
            gain_nm_s_per_rad: 1.0,
            max_effort_nm: 50.0,
        },
    ));
    for _ in 0..180 {
        step_backend(&mut backend, &mut world, physics_world, fixed_dt())?;
    }
    let parent_transform = *world
        .get::<Transform3>(parent)
        .context("joint parent transform missing")?;
    let child_transform = *world
        .get::<Transform3>(child)
        .context("joint child transform missing")?;
    let parent_anchor = parent_transform.translation;
    let child_anchor = child_transform.translation + child_transform.rotation * anchor_child_m;
    let anchor_error_m = parent_anchor.distance(child_anchor);
    let joint_angle_abs_rad = world
        .get::<JointState>(child)
        .and_then(|state| state.position_rad())
        .context("backend did not synchronize a revolute JointState")?
        .abs();
    let metrics = vec![
        metric(
            "anchor_error_m",
            anchor_error_m,
            0.0,
            "revolute_anchor_m_v1",
        ),
        metric(
            "joint_angle_abs_rad",
            joint_angle_abs_rad,
            0.2,
            "revolute_limit_rad_v1",
        ),
    ];
    let contacts = if backend
        .capabilities()
        .contains(&PhysicsCapability::ContactForce)
    {
        backend.contacts(physics_world)?.to_vec()
    } else {
        Vec::new()
    };
    let snapshot = capture_physics_snapshot(&world, &contacts, 180, fixed_dt().ticks() * 180)?;
    Ok(CaseReport {
        id: capability_case_id(backend_id, PhysicsCapability::Articulation),
        backend: backend_id.to_string(),
        capability: PhysicsCapability::Articulation,
        passed: metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(snapshot.stable_hash()),
        metrics,
        detail: "revolute motor pushes against a bounded joint limit".to_string(),
    })
}

fn run_joint_effort_measurement_case<B: PhysicsBackend>(
    mut backend: B,
    backend_id: &str,
) -> anyhow::Result<CaseReport> {
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::ZERO,
        solver_iterations: 16,
    })?;
    let mut world = World::new();
    let parent = spawn_named(&mut world, "joint_effort_parent");
    world.entity_mut(parent).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        MultibodyLink,
        Transform3::default(),
    ));
    let child = spawn_named(&mut world, "joint_effort_child");
    world.entity_mut(child).insert((
        RigidBody::default(),
        Collider::sphere(0.05),
        MultibodyLink,
        Transform3::from_translation_rotation(Vec3::new(0.0, -1.0, 0.0), Quat::IDENTITY),
        RevoluteJointDesc {
            parent,
            axis: Vec3::Z,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::new(0.0, 1.0, 0.0),
            lower_rad: None,
            upper_rad: None,
        },
        rne_physics::JointActuation::RevoluteEffort {
            effort_nm: 2.0,
            max_effort_nm: 2.0,
        },
    ));
    for _ in 0..30 {
        step_backend(&mut backend, &mut world, physics_world, fixed_dt())?;
    }
    let measured_effort_nm = match world.get::<JointEffortMeasurement>(child) {
        Some(JointEffortMeasurement::Revolute { measured_effort_nm }) => *measured_effort_nm,
        Some(JointEffortMeasurement::Prismatic { .. }) => {
            return Err(anyhow!(
                "backend synchronized a prismatic measurement for a revolute joint"
            ))
        }
        None => {
            return Err(anyhow!(
                "backend did not retain completed-step joint effort"
            ))
        }
    };
    let metrics = vec![metric(
        "measured_effort_nm",
        measured_effort_nm,
        2.0,
        "direct_revolute_effort_nm_v1",
    )];
    Ok(CaseReport {
        id: capability_case_id(backend_id, PhysicsCapability::JointEffortMeasurement),
        backend: backend_id.to_string(),
        capability: PhysicsCapability::JointEffortMeasurement,
        passed: metrics.iter().all(|metric| metric.passed),
        snapshot_hash: None,
        metrics,
        detail: "completed-step native actuator effort is retained as a revolute N*m measurement"
            .to_string(),
    })
}

fn run_contact_case<B: PhysicsBackend>(
    mut backend: B,
    backend_id: &str,
) -> anyhow::Result<CaseReport> {
    let physics_world = backend.create_world(PhysicsWorldDesc::default())?;
    let mut world = World::new();
    let ground = spawn_named(&mut world, "contact_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::new(2.0, 0.5, 2.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let cube = spawn_named(&mut world, "contact_cube");
    let mass_kg = 2.0;
    world.entity_mut(cube).insert((
        RigidBody {
            mass_kg,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::splat(0.25)),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.25, 0.0), Quat::IDENTITY),
    ));
    let mut impulses = Vec::new();
    for step in 0..180 {
        step_backend(&mut backend, &mut world, physics_world, fixed_dt())?;
        if step >= 120 {
            if let Some(contact) = backend.contacts(physics_world)?.iter().find(|contact| {
                (contact.entity_a == ground && contact.entity_b == cube)
                    || (contact.entity_a == cube && contact.entity_b == ground)
            }) {
                impulses.push(contact.impulse as f64);
            }
        }
    }
    if impulses.is_empty() {
        return Err(anyhow!("resting contact did not produce impulse evidence"));
    }
    let measured = impulses.iter().sum::<f64>() / impulses.len() as f64;
    let (effective_mass_kg, tolerance_id) = match backend_id {
        // Rapier currently interprets `RigidBody::mass_kg` as additional mass
        // and adds the cuboid's unit-density 0.5 m ^ 3 shape mass.
        BACKEND_RAPIER => (mass_kg + 0.5_f64.powi(3), "resting_impulse_n_s_v1"),
        #[cfg(feature = "mujoco")]
        BACKEND_MUJOCO => (mass_kg, "mujoco_resting_impulse_n_s_v1"),
        _ => {
            return Err(anyhow!(
                "{backend_id} has no registered contact mass profile"
            ))
        }
    };
    let expected = effective_mass_kg * GRAVITY_M_S2.abs() * fixed_dt().as_seconds().value();
    let metrics = vec![metric(
        "mean_normal_impulse_n_s",
        measured,
        expected,
        tolerance_id,
    )];
    let contacts = backend.contacts(physics_world)?.to_vec();
    let snapshot = capture_physics_snapshot(&world, &contacts, 180, fixed_dt().ticks() * 180)?;
    let pair_is_present = snapshot.contacts.iter().any(|contact| {
        contact.entity_a_index == ground.index().min(cube.index())
            && contact.entity_b_index == ground.index().max(cube.index())
            && contact.normal_impulse_n_s > 0.0
    });
    Ok(CaseReport {
        id: capability_case_id(backend_id, PhysicsCapability::ContactForce),
        backend: backend_id.to_string(),
        capability: PhysicsCapability::ContactForce,
        passed: pair_is_present && metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(snapshot.stable_hash()),
        metrics,
        detail: format!(
            "{} of 60 settled steps reported the canonical load-bearing pair; effective_mass_kg={effective_mass_kg}",
            impulses.len(),
        ),
    })
}

fn run_raycast_case<B: PhysicsBackend>(
    mut backend: B,
    backend_id: &str,
) -> anyhow::Result<CaseReport> {
    let physics_world = backend.create_world(PhysicsWorldDesc::default())?;
    let mut world = World::new();
    spawn_fixed_cuboid(&mut world, "ray_near", Vec3::new(0.0, 0.0, 0.0));
    spawn_fixed_cuboid(&mut world, "ray_far", Vec3::new(0.0, -2.0, 0.0));
    backend.sync_from_ecs(&mut world, physics_world)?;
    let queries = [
        RaycastQuery::downward(Vec3::new(0.0, 5.0, 0.0), 10.0),
        RaycastQuery::downward(Vec3::new(10.0, 5.0, 0.0), 10.0),
    ];
    let first = backend.raycast_batch(physics_world, &queries)?;
    let second = backend.raycast_batch(physics_world, &queries)?;
    let shape_ok = first.len() == 2 && first[0].len() == 2 && first[1].is_empty();
    let ordering_ok = shape_ok && first[0][0].distance_m < first[0][1].distance_m;
    let repeat_ok = first == second;
    let metrics = if shape_ok {
        vec![
            metric(
                "hit_distance_m",
                first[0][0].distance_m,
                4.5,
                "raycast_distance_m_v1",
            ),
            metric(
                "hit_distance_m",
                first[0][1].distance_m,
                6.5,
                "raycast_distance_m_v1",
            ),
        ]
    } else {
        Vec::new()
    };
    Ok(CaseReport {
        id: capability_case_id(backend_id, PhysicsCapability::RaycastBatch),
        backend: backend_id.to_string(),
        capability: PhysicsCapability::RaycastBatch,
        passed: shape_ok && ordering_ok && repeat_ok && metrics.iter().all(|metric| metric.passed),
        snapshot_hash: None,
        metrics,
        detail: format!(
            "query_shape_ok={shape_ok} ordering_ok={ordering_ok} repeat_ok={repeat_ok}"
        ),
    })
}

fn comparison_case(
    left: Result<&FreeFallResult, &anyhow::Error>,
    right: Result<&FreeFallResult, &anyhow::Error>,
    case_id: &str,
    backend_id: &str,
    position_tolerance_id: &str,
    velocity_tolerance_id: &str,
) -> CaseReport {
    match (left, right) {
        (Ok(left), Ok(right)) => {
            let metrics = vec![
                metric(
                    "position_delta_m",
                    (left.position_y_m - right.position_y_m).abs(),
                    0.0,
                    position_tolerance_id,
                ),
                metric(
                    "velocity_delta_m_s",
                    (left.velocity_y_m_s - right.velocity_y_m_s).abs(),
                    0.0,
                    velocity_tolerance_id,
                ),
            ];
            CaseReport {
                id: case_id.to_string(),
                backend: backend_id.to_string(),
                capability: PhysicsCapability::RigidBody,
                passed: metrics.iter().all(|metric| metric.passed),
                snapshot_hash: None,
                metrics,
                detail: "same input vector; approximate solver comparison".to_string(),
            }
        }
        (Err(error), _) | (_, Err(error)) => failed_case(
            case_id,
            backend_id,
            PhysicsCapability::RigidBody,
            error.to_string(),
        ),
    }
}

fn metric(id: &str, measured: f64, expected: f64, tolerance_id: &str) -> MetricReport {
    let spec = tolerance(tolerance_id);
    let absolute_error = (measured - expected).abs();
    MetricReport {
        id: id.to_string(),
        unit: spec.unit.symbol().to_string(),
        measured,
        expected,
        absolute_error,
        tolerance: ToleranceReport {
            id: spec.id.to_string(),
            absolute: spec.absolute,
            relative: spec.relative,
            allowed_error: spec.allowed_error(expected),
            rationale: spec.rationale.to_string(),
        },
        passed: spec.accepts(measured, expected),
    }
}

fn result_case<E: std::fmt::Display>(
    id: &str,
    backend: &str,
    capability: PhysicsCapability,
    result: Result<CaseReport, E>,
) -> CaseReport {
    match result {
        Ok(case) => case,
        Err(error) => failed_case(id, backend, capability, error.to_string()),
    }
}

fn failed_case(
    id: &str,
    backend: &str,
    capability: PhysicsCapability,
    detail: String,
) -> CaseReport {
    CaseReport {
        id: id.to_string(),
        backend: backend.to_string(),
        capability,
        passed: false,
        snapshot_hash: None,
        metrics: Vec::new(),
        detail,
    }
}

fn backend_report(
    manifest: PhysicsBackendManifest,
    runtime_capabilities: Vec<PhysicsCapability>,
    cases: &[CaseReport],
) -> BackendReport {
    let manifest_validation = manifest.validate();
    let runtime_is_canonical =
        canonical_capabilities(&runtime_capabilities) == runtime_capabilities;
    let declarations_match = runtime_capabilities == manifest.capabilities;
    let manifest_passed = manifest_validation.is_ok() && runtime_is_canonical && declarations_match;
    let covered = cases
        .iter()
        .filter(|case| case.backend == manifest.backend_id && case.passed)
        .map(|case| case.capability)
        .collect::<BTreeSet<_>>();
    let covered_capabilities = manifest
        .capabilities
        .iter()
        .filter(|capability| covered.contains(*capability))
        .copied()
        .collect::<Vec<_>>();
    let coverage_passed = covered_capabilities == manifest.capabilities;
    let detail = if let Err(error) = manifest_validation {
        error.to_string()
    } else if !runtime_is_canonical {
        "runtime capabilities are duplicated or not canonically ordered".to_string()
    } else if !declarations_match {
        "manifest capabilities do not match the runtime backend declaration".to_string()
    } else {
        "manifest and runtime capability declarations match".to_string()
    };
    BackendReport {
        manifest,
        runtime_capabilities,
        manifest_passed,
        covered_capabilities,
        coverage_passed,
        detail,
    }
}

fn canonical_capabilities(capabilities: &[PhysicsCapability]) -> Vec<PhysicsCapability> {
    PhysicsCapability::ALL
        .iter()
        .filter(|capability| capabilities.contains(capability))
        .copied()
        .collect()
}

fn step_backend<B: PhysicsBackend>(
    backend: &mut B,
    world: &mut World,
    physics_world: rne_physics::PhysicsWorldId,
    dt: SimDuration,
) -> Result<(), rne_physics::PhysicsError> {
    backend.sync_from_ecs(world, physics_world)?;
    backend.step(physics_world, dt)?;
    backend.sync_to_ecs(world, physics_world)
}

fn fixed_dt() -> SimDuration {
    SimDuration::from_hertz(Hertz::new(STEP_HZ))
}

#[cfg(feature = "mujoco")]
fn mujoco_backend() -> MuJoCoBackend {
    MuJoCoBackend::new(fixed_dt()).expect("MuJoCo 3.9 runtime must be available")
}

fn semi_implicit_free_fall_y() -> f64 {
    let dt_s = fixed_dt().as_seconds().value();
    let steps = FREE_FALL_STEPS as f64;
    FREE_FALL_INITIAL_Y_M + GRAVITY_M_S2 * dt_s * dt_s * steps * (steps + 1.0) / 2.0
}

fn continuous_free_fall_y() -> f64 {
    let time_s = fixed_dt().as_seconds().value() * FREE_FALL_STEPS as f64;
    FREE_FALL_INITIAL_Y_M + 0.5 * GRAVITY_M_S2 * time_s * time_s
}

fn spawn_fixed_cuboid(world: &mut World, name: &str, translation_m: Vec3) -> Entity {
    let entity = spawn_named(world, name);
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: rne_physics::ColliderShape::Cuboid {
                half_extents_m: Vec3::splat(0.5),
            },
            material: PhysicsMaterial::default(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(translation_m, Quat::IDENTITY),
    ));
    entity
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_advertised_capability_has_passing_evidence() {
        let report = run_conformance();
        let failures = report
            .cases
            .iter()
            .filter(|case| !case.passed)
            .map(|case| format!("{}: {}", case.id, case.detail))
            .collect::<Vec<_>>();
        assert!(report.all_passed, "conformance failures: {failures:#?}");
        assert!(report
            .backends
            .iter()
            .all(|backend| backend.manifest_passed && backend.coverage_passed));

        let rapier_effort = report
            .cases
            .iter()
            .find(|case| case.id == "rapier.joint_effort_measurement.direct_revolute_effort")
            .expect("Rapier joint-effort capability case");
        assert!(rapier_effort.passed);
        assert_eq!(rapier_effort.metrics.len(), 1);
        assert_eq!(rapier_effort.metrics[0].unit, "N*m");
        assert_eq!(rapier_effort.metrics[0].measured, 2.0);

        #[cfg(feature = "mujoco")]
        {
            let effort = report
                .cases
                .iter()
                .find(|case| case.id == "mujoco.joint_effort_measurement.direct_revolute_effort")
                .expect("MuJoCo joint-effort capability case");
            assert!(effort.passed);
            assert_eq!(effort.metrics.len(), 1);
            assert_eq!(effort.metrics[0].unit, "N*m");
            assert_eq!(effort.metrics[0].measured, 2.0);
        }
    }

    #[test]
    fn public_v2_runner_accepts_any_backend_factory_and_checks_its_manifest() {
        let valid = run_backend_conformance(AnalyticBackend::manifest(), AnalyticBackend::new);
        assert!(valid.all_passed());

        let mut mismatch = AnalyticBackend::manifest();
        mismatch.capabilities = vec![PhysicsCapability::RigidBody];
        mismatch.repeatability = rne_physics::PhysicsBackendRepeatability::ToleranceBounded;
        mismatch
            .validate()
            .expect("modified manifest is structurally valid");
        let mismatch = run_backend_conformance(mismatch, AnalyticBackend::new);
        assert!(!mismatch.all_passed());
        assert!(!mismatch.backend.manifest_passed);
        assert!(mismatch.backend.detail.contains("do not match the runtime"));
    }

    #[test]
    fn unregistered_backend_tolerance_becomes_explicit_failure() {
        let mut manifest = AnalyticBackend::manifest();
        manifest.backend_id = "third_party".to_string();
        let report = run_backend_conformance(manifest, AnalyticBackend::new);
        assert!(!report.all_passed());
        assert!(report.cases.iter().any(|case| {
            case.id == "third_party.rigid_body.free_fall"
                && !case.passed
                && case.detail.contains("no named free-fall tolerance")
        }));
    }

    #[test]
    fn report_json_is_repeatable_and_sorted() {
        let first = run_conformance();
        let second = run_conformance();
        assert_eq!(first, second);
        assert_eq!(
            serde_json::to_string_pretty(&first).unwrap(),
            serde_json::to_string_pretty(&second).unwrap()
        );
        assert!(first.cases.windows(2).all(|pair| pair[0].id < pair[1].id));
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn divergence_diagnostic_is_repeatable_and_preserves_passing_production_bounds() {
        let (first_report, first_replay) = run_divergence_diagnostic().expect("diagnostic");
        let (second_report, second_replay) = run_divergence_diagnostic().expect("repeat");
        assert_eq!(first_report, second_report);
        assert_eq!(first_replay, second_replay);
        assert!(!first_report.all_passed());
        assert!(first_report
            .cases
            .iter()
            .any(|case| { case.id == CASE_RAPIER_MUJOCO_COMPARISON && case.passed }));
        let diagnostic = first_report
            .cases
            .iter()
            .find(|case| case.id == CASE_RAPIER_MUJOCO_DIAGNOSTIC)
            .expect("diagnostic case");
        assert!(!diagnostic.passed);
        assert!(diagnostic.detail.contains("first exceeded"));
        assert_eq!(first_replay.failure.violation.step, 10);
        assert_eq!(first_replay.failure.contract.name, DIAGNOSTIC_CONTRACT);
        first_replay.validate().expect("valid behavior replay");
    }

    #[test]
    fn tolerance_registry_has_unique_ids_and_explicit_units() {
        let ids = TOLERANCES
            .iter()
            .map(|tolerance| tolerance.id)
            .collect::<BTreeSet<_>>();
        assert_eq!(ids.len(), TOLERANCES.len());
        assert!(TOLERANCES.iter().all(|tolerance| {
            !tolerance.case_id.is_empty()
                && !tolerance.metric_id.is_empty()
                && !tolerance.unit.symbol().is_empty()
                && !tolerance.rationale.is_empty()
                && tolerance.absolute >= 0.0
                && tolerance.relative >= 0.0
        }));
    }

    #[test]
    fn report_v2_schema_matches_committed_golden_and_rejects_unknown_fields() {
        let golden = include_str!("../../golden/physics/conformance-report-v2.json");
        let report: ConformanceReport = serde_json::from_str(golden).expect("parse v2 golden");
        assert_eq!(report.kind, CONFORMANCE_REPORT_KIND);
        assert_eq!(
            report.schema_version,
            PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION
        );
        assert_eq!(report.catalog_version, CONFORMANCE_CATALOG_VERSION);
        assert_eq!(
            serde_json::to_string_pretty(&report).expect("serialize v2 golden"),
            golden.trim_end()
        );

        let mut value = serde_json::to_value(report).expect("report value");
        value["unexpected"] = serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<ConformanceReport>(value).is_err());
    }
}
