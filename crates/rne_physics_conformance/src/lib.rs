//! Public conformance catalog for independently maintained RNE physics backends.
//!
//! The kit consumes only the backend-neutral [`rne_physics::PhysicsBackend`]
//! interface. Native engine handles and vendor types remain inside the backend
//! crate. Every case creates a fresh backend so state cannot leak between
//! capability checks.

#![deny(missing_docs)]

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    capture_physics_snapshot, Collider, JointEffortMeasurement, JointState, MultibodyLink,
    PhysicsBackend, PhysicsBackendManifest, PhysicsCapability, PhysicsMaterial, PhysicsSnapshot,
    PhysicsWorldDesc, PhysicsWorldId, RaycastQuery, RevoluteJointDesc, RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;
use thiserror::Error;

/// Stable kind written by an external physics-backend conformance report.
pub const EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_KIND: &str =
    "rne_external_physics_backend_conformance_report";

/// Current external physics-backend conformance report schema.
pub const EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_SCHEMA_VERSION: u16 = 1;

/// Current immutable external capability-case catalog.
pub const EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_CATALOG_VERSION: u16 = 2;

/// Current named, unit-bearing tolerance registry used by the external catalog.
pub const EXTERNAL_PHYSICS_BACKEND_TOLERANCE_REGISTRY_VERSION: u16 = 2;

const STEP_HZ: f64 = 60.0;
const FREE_FALL_STEPS: u64 = 60;
const GRAVITY_M_S2: f64 = -9.81;
const FREE_FALL_INITIAL_Y_M: f64 = 5.0;

const CHECK_IDS: [&str; 10] = [
    "manifest_identity",
    "rigid_body.free_fall",
    "articulation.revolute_limit",
    "gpu_rigid_body.catalog_unsupported",
    "deterministic_step.repeat_snapshot",
    "soft_body.catalog_unsupported",
    "contact_force.resting_impulse",
    "raycast_batch.ordered_hits",
    "kinematic_body.external_pose",
    "joint_effort_measurement.direct_revolute_effort",
];

/// Invalid conformance configuration, report, or serialized artifact.
#[derive(Debug, Error)]
pub enum ExternalPhysicsBackendConformanceError {
    /// The implementation subject label is empty or contains control characters.
    #[error("external physics backend subject label is invalid")]
    InvalidSubjectLabel,
    /// The implementation digest is not a canonical lowercase SHA-256 value.
    #[error("external physics backend subject digest must be sha256:<64 lowercase hex>")]
    InvalidSubjectDigest,
    /// A report violated its schema-derived invariants.
    #[error("invalid external physics backend conformance report: {0}")]
    InvalidReport(String),
    /// JSON encoding or decoding failed.
    #[error("external physics backend conformance JSON failed: {0}")]
    Json(#[from] serde_json::Error),
    /// Reading or writing a report failed.
    #[error("external physics backend conformance I/O failed: {0}")]
    Io(#[from] std::io::Error),
}

/// Content identity for the independently built backend implementation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalPhysicsBackendSubject {
    /// Stable human-readable artifact or source-bundle label.
    pub label: String,
    /// SHA-256 of the exact implementation artifact or source bundle bytes.
    pub sha256: String,
}

impl ExternalPhysicsBackendSubject {
    /// Hashes exact implementation bytes and creates a validated subject.
    pub fn from_bytes(
        label: impl Into<String>,
        bytes: &[u8],
    ) -> Result<Self, ExternalPhysicsBackendConformanceError> {
        let subject = Self {
            label: label.into(),
            sha256: sha256(bytes),
        };
        subject.validate()?;
        Ok(subject)
    }

    /// Validates the stable label and canonical digest encoding.
    pub fn validate(&self) -> Result<(), ExternalPhysicsBackendConformanceError> {
        if self.label.trim().is_empty() || self.label.chars().any(char::is_control) {
            return Err(ExternalPhysicsBackendConformanceError::InvalidSubjectLabel);
        }
        if !is_sha256(&self.sha256) {
            return Err(ExternalPhysicsBackendConformanceError::InvalidSubjectDigest);
        }
        Ok(())
    }
}

/// Inputs bound into one external backend conformance execution.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExternalPhysicsBackendConformanceConfig {
    /// Content identity of the independently built implementation.
    pub subject: ExternalPhysicsBackendSubject,
    /// Backend-neutral capabilities and repeatability declaration.
    pub manifest: PhysicsBackendManifest,
}

impl ExternalPhysicsBackendConformanceConfig {
    /// Creates a conformance configuration without altering semantic failures.
    pub const fn new(
        subject: ExternalPhysicsBackendSubject,
        manifest: PhysicsBackendManifest,
    ) -> Self {
        Self { subject, manifest }
    }
}

/// Outcome of one catalog check.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExternalPhysicsBackendCheckStatus {
    /// The advertised behavior satisfied the fixed contract.
    Passed,
    /// The advertised behavior failed or could not execute.
    Failed,
    /// The capability was not advertised and therefore was not exercised.
    NotAdvertised,
}

/// Named tolerance copied into every numeric metric.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalPhysicsBackendTolerance {
    /// Stable tolerance identifier.
    pub id: String,
    /// Absolute error allowance in the metric's unit.
    pub absolute: f64,
    /// Relative error allowance as a unitless fraction.
    pub relative: f64,
    /// Effective allowance for this expected value.
    pub allowed_error: f64,
    /// Why this tolerance is wide enough and no wider.
    pub rationale: String,
}

/// One unit-bearing numeric comparison from a catalog check.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalPhysicsBackendMetric {
    /// Stable metric identifier within its check.
    pub id: String,
    /// Explicit SI unit or `1` for a unitless value.
    pub unit: String,
    /// Value produced by the backend.
    pub measured: f64,
    /// Canonical catalog reference value.
    pub expected: f64,
    /// Absolute difference between measured and expected.
    pub absolute_error: f64,
    /// Fixed catalog tolerance applied to this value.
    pub tolerance: ExternalPhysicsBackendTolerance,
    /// Whether the value fell inside the fixed tolerance.
    pub passed: bool,
}

/// One stable external backend conformance check.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalPhysicsBackendCheck {
    /// Stable catalog check identifier.
    pub id: String,
    /// Capability exercised by the check, or `None` for manifest identity.
    pub capability: Option<PhysicsCapability>,
    /// Passed, failed, or not-advertised outcome.
    pub status: ExternalPhysicsBackendCheckStatus,
    /// Stable canonical snapshot hash when the case produces world state.
    pub snapshot_hash: Option<u64>,
    /// Unit-bearing comparisons in deterministic order.
    pub metrics: Vec<ExternalPhysicsBackendMetric>,
    /// Bounded diagnostic that does not participate in pass criteria.
    pub detail: String,
}

/// Content-addressed report for one independently maintained physics backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalPhysicsBackendConformanceReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Report shape version.
    pub schema_version: u16,
    /// Capability-case catalog version.
    pub catalog_version: u16,
    /// Named tolerance registry version.
    pub tolerance_registry_version: u16,
    /// Exact implementation artifact or source-bundle identity.
    pub subject: ExternalPhysicsBackendSubject,
    /// SHA-256 of the canonical serialized backend manifest.
    pub manifest_sha256: String,
    /// Backend capability and repeatability declaration.
    pub manifest: PhysicsBackendManifest,
    /// Capabilities returned by the runtime backend in canonical order.
    pub runtime_capabilities: Vec<PhysicsCapability>,
    /// Advertised capabilities proven by passing cases.
    pub covered_capabilities: Vec<PhysicsCapability>,
    /// Ten catalog checks in fixed order.
    pub checks: Vec<ExternalPhysicsBackendCheck>,
    /// True only when identity and every advertised capability pass.
    pub passed: bool,
}

impl ExternalPhysicsBackendConformanceReport {
    /// Returns the aggregate semantic verdict.
    pub const fn passed(&self) -> bool {
        self.passed
    }

    /// Validates report shape, hashes, check order, metrics, coverage, and verdict.
    pub fn validate(&self) -> Result<(), ExternalPhysicsBackendConformanceError> {
        self.subject.validate()?;
        ensure_report(
            self.kind == EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_KIND,
            "kind mismatch",
        )?;
        ensure_report(
            self.schema_version == EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_SCHEMA_VERSION,
            "schema version mismatch",
        )?;
        ensure_report(
            self.catalog_version == EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_CATALOG_VERSION,
            "catalog version mismatch",
        )?;
        ensure_report(
            self.tolerance_registry_version == EXTERNAL_PHYSICS_BACKEND_TOLERANCE_REGISTRY_VERSION,
            "tolerance registry version mismatch",
        )?;
        ensure_report(
            self.manifest_sha256 == manifest_sha256(&self.manifest)?,
            "manifest digest mismatch",
        )?;
        ensure_report(
            self.checks.len() == CHECK_IDS.len()
                && self
                    .checks
                    .iter()
                    .zip(CHECK_IDS)
                    .all(|(check, id)| check.id == id),
            "catalog check order mismatch",
        )?;
        ensure_report(
            canonical_capabilities(&self.runtime_capabilities) == self.runtime_capabilities,
            "runtime capabilities are not canonical",
        )?;

        for (index, capability) in PhysicsCapability::ALL.iter().copied().enumerate() {
            let check = &self.checks[index + 1];
            ensure_report(
                check.capability == Some(capability),
                "catalog capability does not match its check",
            )?;
            let advertised = self.manifest.capabilities.contains(&capability);
            ensure_report(
                advertised != (check.status == ExternalPhysicsBackendCheckStatus::NotAdvertised),
                "advertised capability and check status disagree",
            )?;
        }
        ensure_report(
            self.checks[0].capability.is_none()
                && self.checks[0].status != ExternalPhysicsBackendCheckStatus::NotAdvertised,
            "manifest identity check is malformed",
        )?;
        let manifest_passed = self.manifest.validate().is_ok()
            && self.runtime_capabilities == self.manifest.capabilities;
        ensure_report(
            self.checks[0].status == status(manifest_passed),
            "manifest identity verdict mismatch",
        )?;
        for check in &self.checks {
            ensure_report(check.detail.len() <= 4_096, "check diagnostic is too large")?;
            for metric in &check.metrics {
                validate_metric(metric)?;
            }
            ensure_report(
                check.status != ExternalPhysicsBackendCheckStatus::Passed
                    || check.metrics.iter().all(|metric| metric.passed),
                "passing check contains a failed metric",
            )?;
            if matches!(
                check.capability,
                Some(PhysicsCapability::GpuRigidBody | PhysicsCapability::SoftBody)
            ) && self
                .manifest
                .capabilities
                .iter()
                .any(|capability| Some(*capability) == check.capability)
            {
                ensure_report(
                    check.status == ExternalPhysicsBackendCheckStatus::Failed,
                    "catalog-unsupported capability must fail closed",
                )?;
            }
        }

        let covered = PhysicsCapability::ALL
            .iter()
            .copied()
            .filter(|capability| {
                self.checks
                    .iter()
                    .find(|check| check.capability == Some(*capability))
                    .is_some_and(|check| check.status == ExternalPhysicsBackendCheckStatus::Passed)
            })
            .collect::<Vec<_>>();
        ensure_report(
            covered == self.covered_capabilities,
            "covered capabilities do not match passing checks",
        )?;
        let derived_passed = self.checks[0].status == ExternalPhysicsBackendCheckStatus::Passed
            && self
                .checks
                .iter()
                .skip(1)
                .all(|check| check.status != ExternalPhysicsBackendCheckStatus::Failed);
        ensure_report(self.passed == derived_passed, "aggregate verdict mismatch")
    }

    /// Serializes a validated report as stable pretty JSON with a trailing newline.
    pub fn to_json_pretty(&self) -> Result<String, ExternalPhysicsBackendConformanceError> {
        self.validate()?;
        let mut json = serde_json::to_string_pretty(self)?;
        json.push('\n');
        Ok(json)
    }

    /// Writes a validated report as pretty JSON.
    pub fn write_json(
        &self,
        path: impl AsRef<Path>,
    ) -> Result<(), ExternalPhysicsBackendConformanceError> {
        fs::write(path, self.to_json_pretty()?)?;
        Ok(())
    }

    /// Reads and validates a report from JSON.
    pub fn read_json(
        path: impl AsRef<Path>,
    ) -> Result<Self, ExternalPhysicsBackendConformanceError> {
        let report: Self = serde_json::from_slice(&fs::read(path)?)?;
        report.validate()?;
        Ok(report)
    }
}

/// Runs the immutable external capability catalog against a fresh backend per case.
///
/// Semantic failures are returned in a valid failed report. Configuration and
/// report-shape failures are returned as errors. `GpuRigidBody` and `SoftBody`
/// currently fail closed because catalog v1 has no portable vector for them.
pub fn run_external_backend_conformance<B, F>(
    config: ExternalPhysicsBackendConformanceConfig,
    factory: F,
) -> Result<ExternalPhysicsBackendConformanceReport, ExternalPhysicsBackendConformanceError>
where
    B: PhysicsBackend,
    F: Fn() -> B,
{
    config.subject.validate()?;
    let runtime_capabilities = factory().capabilities().to_vec();
    let manifest_ok = config.manifest.validate().is_ok()
        && canonical_capabilities(&runtime_capabilities) == runtime_capabilities
        && runtime_capabilities == config.manifest.capabilities;
    let mut checks = Vec::with_capacity(CHECK_IDS.len());
    checks.push(ExternalPhysicsBackendCheck {
        id: CHECK_IDS[0].to_string(),
        capability: None,
        status: status(manifest_ok),
        snapshot_hash: None,
        metrics: Vec::new(),
        detail: manifest_detail(&config.manifest, &runtime_capabilities),
    });

    for capability in PhysicsCapability::ALL {
        if !config.manifest.capabilities.contains(&capability) {
            checks.push(not_advertised(capability));
            continue;
        }
        let check = match capability {
            PhysicsCapability::RigidBody => case_result(capability, run_free_fall(factory())),
            PhysicsCapability::Articulation => case_result(capability, run_articulation(factory())),
            PhysicsCapability::GpuRigidBody => unsupported_case(
                capability,
                "catalog v1 has no portable GPU rigid-body vector",
            ),
            PhysicsCapability::DeterministicStep => run_determinism(&factory),
            PhysicsCapability::SoftBody => {
                unsupported_case(capability, "catalog v1 has no portable soft-body vector")
            }
            PhysicsCapability::ContactForce => case_result(capability, run_contact(factory())),
            PhysicsCapability::RaycastBatch => case_result(capability, run_raycast(factory())),
            PhysicsCapability::KinematicBody => case_result(capability, run_kinematic(factory())),
            PhysicsCapability::JointEffortMeasurement => {
                case_result(capability, run_joint_effort_measurement(factory()))
            }
        };
        checks.push(check);
    }

    let covered_capabilities = PhysicsCapability::ALL
        .iter()
        .copied()
        .filter(|capability| {
            checks
                .iter()
                .find(|check| check.capability == Some(*capability))
                .is_some_and(|check| check.status == ExternalPhysicsBackendCheckStatus::Passed)
        })
        .collect::<Vec<_>>();
    let passed = checks[0].status == ExternalPhysicsBackendCheckStatus::Passed
        && checks
            .iter()
            .skip(1)
            .all(|check| check.status != ExternalPhysicsBackendCheckStatus::Failed);
    let report = ExternalPhysicsBackendConformanceReport {
        kind: EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_KIND.to_string(),
        schema_version: EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_REPORT_SCHEMA_VERSION,
        catalog_version: EXTERNAL_PHYSICS_BACKEND_CONFORMANCE_CATALOG_VERSION,
        tolerance_registry_version: EXTERNAL_PHYSICS_BACKEND_TOLERANCE_REGISTRY_VERSION,
        subject: config.subject,
        manifest_sha256: manifest_sha256(&config.manifest)?,
        manifest: config.manifest,
        runtime_capabilities,
        covered_capabilities,
        checks,
        passed,
    };
    report.validate()?;
    Ok(report)
}

#[derive(Clone, Debug)]
struct FreeFallEvidence {
    position_y_m: f64,
    velocity_y_m_s: f64,
    snapshot: PhysicsSnapshot,
}

fn run_free_fall<B: PhysicsBackend>(mut backend: B) -> Result<ExternalPhysicsBackendCheck, String> {
    let mut world = World::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, GRAVITY_M_S2, 0.0),
            ..PhysicsWorldDesc::default()
        })
        .map_err(|error| error.to_string())?;
    let body = spawn_named(&mut world, "external_free_fall_body");
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
    let evidence = free_fall_evidence(&mut backend, &mut world, physics_world, body)?;
    let metrics = vec![
        metric(
            "position_y_m",
            "m",
            evidence.position_y_m,
            continuous_free_fall_y(),
            "external_free_fall_position_m_v1",
            0.10,
            0.0,
            "common explicit, semi-implicit, and production solver integration conventions differ by at most one bounded O(dt) term",
        ),
        metric(
            "velocity_y_m_s",
            "m/s",
            evidence.velocity_y_m_s,
            GRAVITY_M_S2,
            "external_free_fall_velocity_m_s_v1",
            0.001,
            0.0,
            "constant acceleration permits only numeric pipeline rounding after one second",
        ),
    ];
    Ok(completed_case(
        PhysicsCapability::RigidBody,
        metrics.iter().all(|metric| metric.passed),
        Some(evidence.snapshot.stable_hash()),
        metrics,
        "shared 60 Hz free-fall vector",
    ))
}

fn free_fall_evidence<B: PhysicsBackend>(
    backend: &mut B,
    world: &mut World,
    physics_world: PhysicsWorldId,
    body: Entity,
) -> Result<FreeFallEvidence, String> {
    let dt = fixed_dt();
    for _ in 0..FREE_FALL_STEPS {
        step_backend(backend, world, physics_world, dt)?;
    }
    let contacts = if backend
        .capabilities()
        .contains(&PhysicsCapability::ContactForce)
    {
        backend
            .contacts(physics_world)
            .map_err(|error| error.to_string())?
            .to_vec()
    } else {
        Vec::new()
    };
    let snapshot = capture_physics_snapshot(
        world,
        &contacts,
        FREE_FALL_STEPS,
        dt.ticks() * FREE_FALL_STEPS,
    )
    .map_err(|error| error.to_string())?;
    let transform = world
        .get::<Transform3>(body)
        .ok_or_else(|| "free-fall transform missing".to_string())?;
    let rigid_body = world
        .get::<RigidBody>(body)
        .ok_or_else(|| "free-fall rigid body missing".to_string())?;
    Ok(FreeFallEvidence {
        position_y_m: transform.translation.y,
        velocity_y_m_s: rigid_body.linear_velocity_m_s.y,
        snapshot,
    })
}

fn run_determinism<B, F>(factory: &F) -> ExternalPhysicsBackendCheck
where
    B: PhysicsBackend,
    F: Fn() -> B,
{
    let first = free_fall_snapshot(factory());
    let second = free_fall_snapshot(factory());
    match (first, second) {
        (Ok(first), Ok(second)) => completed_case(
            PhysicsCapability::DeterministicStep,
            first == second && first.stable_hash() == second.stable_hash(),
            Some(first.stable_hash()),
            Vec::new(),
            if first == second {
                "two fresh executions produced identical canonical snapshots"
            } else {
                "fresh executions produced different canonical snapshots"
            },
        ),
        (Err(error), _) | (_, Err(error)) => failed_case(
            PhysicsCapability::DeterministicStep,
            format!("repeatability vector failed: {error}"),
        ),
    }
}

fn free_fall_snapshot<B: PhysicsBackend>(mut backend: B) -> Result<PhysicsSnapshot, String> {
    let mut world = World::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, GRAVITY_M_S2, 0.0),
            ..PhysicsWorldDesc::default()
        })
        .map_err(|error| error.to_string())?;
    let body = spawn_named(&mut world, "external_repeat_body");
    world.entity_mut(body).insert((
        RigidBody::default(),
        Collider::sphere(0.05),
        Transform3::from_translation_rotation(
            Vec3::new(0.0, FREE_FALL_INITIAL_Y_M, 0.0),
            Quat::IDENTITY,
        ),
    ));
    free_fall_evidence(&mut backend, &mut world, physics_world, body)
        .map(|evidence| evidence.snapshot)
}

fn run_kinematic<B: PhysicsBackend>(mut backend: B) -> Result<ExternalPhysicsBackendCheck, String> {
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            ..PhysicsWorldDesc::default()
        })
        .map_err(|error| error.to_string())?;
    let mut world = World::new();
    let body = spawn_named(&mut world, "external_kinematic_body");
    world.entity_mut(body).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        Collider::sphere(0.1),
        Transform3::default(),
    ));
    step_backend(&mut backend, &mut world, physics_world, fixed_dt())?;
    let target = Transform3::from_translation_rotation(
        Vec3::new(1.25, -0.5, 2.75),
        Quat::from_rotation_z(0.3),
    );
    *world
        .get_mut::<Transform3>(body)
        .ok_or_else(|| "kinematic transform missing before external pose".to_string())? = target;
    step_backend(&mut backend, &mut world, physics_world, fixed_dt())?;
    let actual = *world
        .get::<Transform3>(body)
        .ok_or_else(|| "kinematic transform missing after step".to_string())?;
    let rotation_dot = actual.rotation.dot(target.rotation).abs().clamp(0.0, 1.0);
    let metrics = vec![
        metric(
            "translation_error_m",
            "m",
            actual.translation.distance(target.translation),
            0.0,
            "external_kinematic_translation_m_v1",
            1e-6,
            0.0,
            "externally supplied translation permits only f32 conversion rounding",
        ),
        metric(
            "rotation_error_rad",
            "rad",
            2.0 * rotation_dot.acos(),
            0.0,
            "external_kinematic_rotation_rad_v1",
            1e-6,
            0.0,
            "externally supplied rotation permits only normalized f32 conversion rounding",
        ),
    ];
    let snapshot = snapshot(&backend, &world, physics_world, 2)?;
    Ok(completed_case(
        PhysicsCapability::KinematicBody,
        metrics.iter().all(|metric| metric.passed),
        Some(snapshot.stable_hash()),
        metrics,
        "externally supplied pose remains authoritative across a fixed step",
    ))
}

fn run_articulation<B: PhysicsBackend>(
    mut backend: B,
) -> Result<ExternalPhysicsBackendCheck, String> {
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            solver_iterations: 16,
        })
        .map_err(|error| error.to_string())?;
    let mut world = World::new();
    let parent = spawn_named(&mut world, "external_joint_parent");
    world.entity_mut(parent).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        MultibodyLink,
        Transform3::default(),
    ));
    let child = spawn_named(&mut world, "external_joint_child");
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
        .ok_or_else(|| "joint parent transform missing".to_string())?;
    let child_transform = *world
        .get::<Transform3>(child)
        .ok_or_else(|| "joint child transform missing".to_string())?;
    let joint_angle_abs_rad = world
        .get::<JointState>(child)
        .and_then(|state| state.position_rad())
        .ok_or_else(|| "backend did not synchronize a revolute JointState".to_string())?
        .abs();
    let metrics = vec![
        metric(
            "anchor_error_m",
            "m",
            parent_transform
                .translation
                .distance(child_transform.translation + child_transform.rotation * anchor_child_m),
            0.0,
            "external_revolute_anchor_m_v1",
            0.01,
            0.0,
            "iterative constraint stabilization may leave millimetre anchor error",
        ),
        metric(
            "joint_angle_abs_rad",
            "rad",
            joint_angle_abs_rad,
            0.2,
            "external_revolute_limit_rad_v1",
            0.03,
            0.0,
            "motor load and discrete limit stabilization permit 0.03 rad slack",
        ),
    ];
    let snapshot = snapshot(&backend, &world, physics_world, 180)?;
    Ok(completed_case(
        PhysicsCapability::Articulation,
        metrics.iter().all(|metric| metric.passed),
        Some(snapshot.stable_hash()),
        metrics,
        "revolute motor pushes against a bounded joint limit",
    ))
}

fn run_joint_effort_measurement<B: PhysicsBackend>(
    mut backend: B,
) -> Result<ExternalPhysicsBackendCheck, String> {
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            solver_iterations: 16,
        })
        .map_err(|error| error.to_string())?;
    let mut world = World::new();
    let parent = spawn_named(&mut world, "external_effort_parent");
    world.entity_mut(parent).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        MultibodyLink,
        Transform3::default(),
    ));
    let child = spawn_named(&mut world, "external_effort_child");
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
            return Err(
                "backend synchronized a prismatic measurement for a revolute joint".to_string(),
            )
        }
        None => return Err("backend did not retain completed-step joint effort".to_string()),
    };
    let metrics = vec![metric(
        "measured_effort_nm",
        "N*m",
        measured_effort_nm,
        2.0,
        "external_direct_revolute_effort_nm_v1",
        1e-9,
        1e-9,
        "direct actuator effort must retain its commanded SI value apart from numeric conversion rounding",
    )];
    Ok(completed_case(
        PhysicsCapability::JointEffortMeasurement,
        metrics.iter().all(|metric| metric.passed),
        None,
        metrics,
        "completed-step native actuator effort is retained as a revolute N*m measurement",
    ))
}

fn run_contact<B: PhysicsBackend>(mut backend: B) -> Result<ExternalPhysicsBackendCheck, String> {
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .map_err(|error| error.to_string())?;
    let mut world = World::new();
    let ground = spawn_named(&mut world, "external_contact_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::new(2.0, 0.5, 2.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let cube = spawn_named(&mut world, "external_contact_cube");
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
            let contacts = backend
                .contacts(physics_world)
                .map_err(|error| error.to_string())?;
            if let Some(contact) = contacts.iter().find(|contact| {
                (contact.entity_a == ground && contact.entity_b == cube)
                    || (contact.entity_a == cube && contact.entity_b == ground)
            }) {
                impulses.push(f64::from(contact.impulse));
            }
        }
    }
    if impulses.is_empty() {
        return Err("resting contact did not produce impulse evidence".to_string());
    }
    let measured = impulses.iter().sum::<f64>() / impulses.len() as f64;
    let expected = mass_kg * GRAVITY_M_S2.abs() * fixed_dt().as_seconds().value();
    let metrics = vec![metric(
        "mean_normal_impulse_n_s",
        measured_unit(),
        measured,
        expected,
        "external_resting_impulse_n_s_v1",
        0.015,
        0.35,
        "settled iterative contact manifolds may bias impulse around body weight integrated over one step",
    )];
    let snapshot = snapshot(&backend, &world, physics_world, 180)?;
    let pair_present = snapshot.contacts.iter().any(|contact| {
        contact.entity_a_index == ground.index().min(cube.index())
            && contact.entity_b_index == ground.index().max(cube.index())
            && contact.normal_impulse_n_s > 0.0
    });
    Ok(completed_case(
        PhysicsCapability::ContactForce,
        pair_present && metrics.iter().all(|metric| metric.passed),
        Some(snapshot.stable_hash()),
        metrics,
        &format!(
            "{} of 60 settled steps reported the canonical load-bearing pair",
            impulses.len()
        ),
    ))
}

fn run_raycast<B: PhysicsBackend>(mut backend: B) -> Result<ExternalPhysicsBackendCheck, String> {
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .map_err(|error| error.to_string())?;
    let mut world = World::new();
    spawn_fixed_cuboid(&mut world, "external_ray_near", Vec3::ZERO);
    spawn_fixed_cuboid(&mut world, "external_ray_far", Vec3::new(0.0, -2.0, 0.0));
    backend
        .sync_from_ecs(&mut world, physics_world)
        .map_err(|error| error.to_string())?;
    let queries = [
        RaycastQuery::downward(Vec3::new(0.0, 5.0, 0.0), 10.0),
        RaycastQuery::downward(Vec3::new(10.0, 5.0, 0.0), 10.0),
    ];
    let first = backend
        .raycast_batch(physics_world, &queries)
        .map_err(|error| error.to_string())?;
    let second = backend
        .raycast_batch(physics_world, &queries)
        .map_err(|error| error.to_string())?;
    let shape_ok = first.len() == 2 && first[0].len() == 2 && first[1].is_empty();
    let ordering_ok = shape_ok && first[0][0].distance_m < first[0][1].distance_m;
    let repeat_ok = first == second;
    let metrics = if shape_ok {
        vec![
            metric(
                "near_hit_distance_m",
                "m",
                first[0][0].distance_m,
                4.5,
                "external_raycast_distance_m_v1",
                1e-5,
                0.0,
                "axis-aligned cuboid intersections permit only f32 conversion rounding",
            ),
            metric(
                "far_hit_distance_m",
                "m",
                first[0][1].distance_m,
                6.5,
                "external_raycast_distance_m_v1",
                1e-5,
                0.0,
                "axis-aligned cuboid intersections permit only f32 conversion rounding",
            ),
        ]
    } else {
        Vec::new()
    };
    Ok(completed_case(
        PhysicsCapability::RaycastBatch,
        shape_ok && ordering_ok && repeat_ok && metrics.iter().all(|metric| metric.passed),
        None,
        metrics,
        &format!("query_shape_ok={shape_ok} ordering_ok={ordering_ok} repeat_ok={repeat_ok}"),
    ))
}

fn snapshot<B: PhysicsBackend>(
    backend: &B,
    world: &World,
    physics_world: PhysicsWorldId,
    steps: u64,
) -> Result<PhysicsSnapshot, String> {
    let contacts = if backend
        .capabilities()
        .contains(&PhysicsCapability::ContactForce)
    {
        backend
            .contacts(physics_world)
            .map_err(|error| error.to_string())?
            .to_vec()
    } else {
        Vec::new()
    };
    capture_physics_snapshot(world, &contacts, steps, fixed_dt().ticks() * steps)
        .map_err(|error| error.to_string())
}

fn step_backend<B: PhysicsBackend>(
    backend: &mut B,
    world: &mut World,
    physics_world: PhysicsWorldId,
    dt: SimDuration,
) -> Result<(), String> {
    backend
        .sync_from_ecs(world, physics_world)
        .map_err(|error| error.to_string())?;
    backend
        .step(physics_world, dt)
        .map_err(|error| error.to_string())?;
    backend
        .sync_to_ecs(world, physics_world)
        .map_err(|error| error.to_string())
}

fn case_result(
    capability: PhysicsCapability,
    result: Result<ExternalPhysicsBackendCheck, String>,
) -> ExternalPhysicsBackendCheck {
    match result {
        Ok(check) => check,
        Err(error) => failed_case(capability, error),
    }
}

fn completed_case(
    capability: PhysicsCapability,
    passed: bool,
    snapshot_hash: Option<u64>,
    metrics: Vec<ExternalPhysicsBackendMetric>,
    detail: &str,
) -> ExternalPhysicsBackendCheck {
    ExternalPhysicsBackendCheck {
        id: capability_check_id(capability).to_string(),
        capability: Some(capability),
        status: status(passed),
        snapshot_hash,
        metrics,
        detail: detail.to_string(),
    }
}

fn failed_case(capability: PhysicsCapability, detail: String) -> ExternalPhysicsBackendCheck {
    completed_case(capability, false, None, Vec::new(), &detail)
}

fn unsupported_case(capability: PhysicsCapability, detail: &str) -> ExternalPhysicsBackendCheck {
    failed_case(capability, detail.to_string())
}

fn not_advertised(capability: PhysicsCapability) -> ExternalPhysicsBackendCheck {
    ExternalPhysicsBackendCheck {
        id: capability_check_id(capability).to_string(),
        capability: Some(capability),
        status: ExternalPhysicsBackendCheckStatus::NotAdvertised,
        snapshot_hash: None,
        metrics: Vec::new(),
        detail: "capability not advertised by the backend manifest".to_string(),
    }
}

fn capability_check_id(capability: PhysicsCapability) -> &'static str {
    match capability {
        PhysicsCapability::RigidBody => CHECK_IDS[1],
        PhysicsCapability::Articulation => CHECK_IDS[2],
        PhysicsCapability::GpuRigidBody => CHECK_IDS[3],
        PhysicsCapability::DeterministicStep => CHECK_IDS[4],
        PhysicsCapability::SoftBody => CHECK_IDS[5],
        PhysicsCapability::ContactForce => CHECK_IDS[6],
        PhysicsCapability::RaycastBatch => CHECK_IDS[7],
        PhysicsCapability::KinematicBody => CHECK_IDS[8],
        PhysicsCapability::JointEffortMeasurement => CHECK_IDS[9],
    }
}

#[allow(clippy::too_many_arguments)]
fn metric(
    id: &str,
    unit: &str,
    measured: f64,
    expected: f64,
    tolerance_id: &str,
    absolute: f64,
    relative: f64,
    rationale: &str,
) -> ExternalPhysicsBackendMetric {
    let absolute_error = (measured - expected).abs();
    let allowed_error = absolute.max(relative * expected.abs());
    ExternalPhysicsBackendMetric {
        id: id.to_string(),
        unit: unit.to_string(),
        measured,
        expected,
        absolute_error,
        tolerance: ExternalPhysicsBackendTolerance {
            id: tolerance_id.to_string(),
            absolute,
            relative,
            allowed_error,
            rationale: rationale.to_string(),
        },
        passed: measured.is_finite() && expected.is_finite() && absolute_error <= allowed_error,
    }
}

fn validate_metric(
    metric: &ExternalPhysicsBackendMetric,
) -> Result<(), ExternalPhysicsBackendConformanceError> {
    let finite = metric.measured.is_finite()
        && metric.expected.is_finite()
        && metric.absolute_error.is_finite()
        && metric.tolerance.absolute.is_finite()
        && metric.tolerance.relative.is_finite()
        && metric.tolerance.allowed_error.is_finite();
    ensure_report(finite, "metric contains non-finite value")?;
    ensure_report(
        metric.tolerance.absolute >= 0.0 && metric.tolerance.relative >= 0.0,
        "metric tolerance is negative",
    )?;
    let absolute_error = (metric.measured - metric.expected).abs();
    let allowed_error = metric
        .tolerance
        .absolute
        .max(metric.tolerance.relative * metric.expected.abs());
    if !derived_float_matches(metric.absolute_error, absolute_error)
        || !derived_float_matches(metric.tolerance.allowed_error, allowed_error)
    {
        return Err(ExternalPhysicsBackendConformanceError::InvalidReport(
            format!(
                "metric {} derived values mismatch: absolute_error={} derived={absolute_error} allowed_error={} derived={allowed_error}",
                metric.id, metric.absolute_error, metric.tolerance.allowed_error
            ),
        ));
    }
    ensure_report(
        metric.passed == (metric.absolute_error <= metric.tolerance.allowed_error),
        "metric verdict mismatch",
    )
}

fn derived_float_matches(recorded: f64, derived: f64) -> bool {
    (recorded - derived).abs() <= f64::EPSILON * recorded.abs().max(derived.abs()).max(1.0) * 4.0
}

fn manifest_detail(
    manifest: &PhysicsBackendManifest,
    runtime_capabilities: &[PhysicsCapability],
) -> String {
    if let Err(error) = manifest.validate() {
        error.to_string()
    } else if canonical_capabilities(runtime_capabilities) != runtime_capabilities {
        "runtime capabilities are duplicated or not canonically ordered".to_string()
    } else if runtime_capabilities != manifest.capabilities {
        "manifest capabilities do not match the runtime backend declaration".to_string()
    } else {
        "manifest and runtime capability declarations match".to_string()
    }
}

fn canonical_capabilities(capabilities: &[PhysicsCapability]) -> Vec<PhysicsCapability> {
    let present = capabilities.iter().copied().collect::<BTreeSet<_>>();
    PhysicsCapability::ALL
        .iter()
        .filter(|capability| present.contains(capability))
        .copied()
        .collect()
}

fn manifest_sha256(
    manifest: &PhysicsBackendManifest,
) -> Result<String, ExternalPhysicsBackendConformanceError> {
    Ok(sha256(&serde_json::to_vec(manifest)?))
}

fn sha256(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut value = String::with_capacity(71);
    value.push_str("sha256:");
    for byte in digest {
        write!(&mut value, "{byte:02x}").expect("writing to String cannot fail");
    }
    value
}

fn is_sha256(value: &str) -> bool {
    value.len() == 71
        && value.starts_with("sha256:")
        && value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn ensure_report(
    condition: bool,
    detail: &str,
) -> Result<(), ExternalPhysicsBackendConformanceError> {
    if condition {
        Ok(())
    } else {
        Err(ExternalPhysicsBackendConformanceError::InvalidReport(
            detail.to_string(),
        ))
    }
}

fn status(passed: bool) -> ExternalPhysicsBackendCheckStatus {
    if passed {
        ExternalPhysicsBackendCheckStatus::Passed
    } else {
        ExternalPhysicsBackendCheckStatus::Failed
    }
}

fn fixed_dt() -> SimDuration {
    SimDuration::from_hertz(Hertz::new(STEP_HZ))
}

fn continuous_free_fall_y() -> f64 {
    let time_s = fixed_dt().as_seconds().value() * FREE_FALL_STEPS as f64;
    FREE_FALL_INITIAL_Y_M + 0.5 * GRAVITY_M_S2 * time_s * time_s
}

fn measured_unit() -> &'static str {
    "N*s"
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
    use rne_physics::{PhysicsBackendRepeatability, PhysicsCapability};
    use rne_physics_analytic::AnalyticBackend;

    fn external_manifest() -> PhysicsBackendManifest {
        PhysicsBackendManifest::new(
            "third_party_analytic_fixture",
            "1.2.3",
            "independent_ballistic_engine",
            "4.5.6",
            [
                PhysicsCapability::RigidBody,
                PhysicsCapability::DeterministicStep,
                PhysicsCapability::KinematicBody,
            ],
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect("external manifest")
    }

    fn config(manifest: PhysicsBackendManifest) -> ExternalPhysicsBackendConformanceConfig {
        ExternalPhysicsBackendConformanceConfig::new(
            ExternalPhysicsBackendSubject::from_bytes(
                "third-party-backend-source.tar.zst",
                b"independently maintained backend fixture v1",
            )
            .expect("subject"),
            manifest,
        )
    }

    #[test]
    fn arbitrary_backend_identity_passes_fixed_portable_catalog() {
        let report = run_external_backend_conformance::<AnalyticBackend, _>(
            config(external_manifest()),
            AnalyticBackend::new,
        )
        .expect("conformance report");
        assert!(report.passed());
        assert_eq!(report.checks.len(), 10);
        assert_eq!(
            report.covered_capabilities,
            external_manifest().capabilities
        );
        report.validate().expect("valid report");
    }

    #[test]
    fn repeated_fresh_runs_are_byte_identical() {
        let first = run_external_backend_conformance::<AnalyticBackend, _>(
            config(external_manifest()),
            AnalyticBackend::new,
        )
        .expect("first report")
        .to_json_pretty()
        .expect("first JSON");
        let second = run_external_backend_conformance::<AnalyticBackend, _>(
            config(external_manifest()),
            AnalyticBackend::new,
        )
        .expect("second report")
        .to_json_pretty()
        .expect("second JSON");
        assert_eq!(first, second);
    }

    #[test]
    fn capability_overclaim_is_a_valid_failed_report() {
        let manifest = PhysicsBackendManifest::new(
            "third_party_overclaim_fixture",
            "1.0.0",
            "independent_ballistic_engine",
            "1.0.0",
            [
                PhysicsCapability::RigidBody,
                PhysicsCapability::GpuRigidBody,
                PhysicsCapability::DeterministicStep,
                PhysicsCapability::KinematicBody,
            ],
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect("overclaim manifest remains structurally valid");
        let report = run_external_backend_conformance::<AnalyticBackend, _>(
            config(manifest),
            AnalyticBackend::new,
        )
        .expect("semantic failure is a report");
        assert!(!report.passed());
        assert_eq!(
            report.checks[3].status,
            ExternalPhysicsBackendCheckStatus::Failed
        );
        report.validate().expect("failed report is valid");
    }

    #[test]
    fn malformed_subject_and_report_shape_are_rejected() {
        let subject = ExternalPhysicsBackendSubject {
            label: "fixture".to_string(),
            sha256: "sha256:ABC".to_string(),
        };
        assert!(matches!(
            subject.validate(),
            Err(ExternalPhysicsBackendConformanceError::InvalidSubjectDigest)
        ));

        let mut report = run_external_backend_conformance::<AnalyticBackend, _>(
            config(external_manifest()),
            AnalyticBackend::new,
        )
        .expect("report");
        report.checks.swap(1, 2);
        assert!(matches!(
            report.validate(),
            Err(ExternalPhysicsBackendConformanceError::InvalidReport(_))
        ));
    }

    #[test]
    fn report_json_denies_unknown_fields() {
        let report = run_external_backend_conformance::<AnalyticBackend, _>(
            config(external_manifest()),
            AnalyticBackend::new,
        )
        .expect("report");
        let mut value = serde_json::to_value(report).expect("report value");
        value
            .as_object_mut()
            .expect("report object")
            .insert("future_field".to_string(), serde_json::json!(true));
        assert!(serde_json::from_value::<ExternalPhysicsBackendConformanceReport>(value).is_err());
    }

    #[test]
    fn report_schema_matches_committed_golden() {
        let subject = ExternalPhysicsBackendSubject::from_bytes(
            "reference-external-backend-source.tar.zst",
            b"reference external backend source bundle v1",
        )
        .expect("subject");
        let manifest = PhysicsBackendManifest::new(
            "reference_external_backend",
            "0.1.0",
            "independent_ballistic_engine",
            "1",
            [
                PhysicsCapability::RigidBody,
                PhysicsCapability::DeterministicStep,
                PhysicsCapability::KinematicBody,
            ],
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect("manifest");
        let report = run_external_backend_conformance::<AnalyticBackend, _>(
            ExternalPhysicsBackendConformanceConfig::new(subject, manifest),
            AnalyticBackend::new,
        )
        .expect("report");
        assert_eq!(
            report.to_json_pretty().expect("report JSON"),
            include_str!("../tests/golden/external-backend-conformance-v1.json")
        );
    }
}
