//! Backend-neutral physics conformance vectors and deterministic reports.

#![deny(missing_docs)]

use anyhow::{anyhow, Context};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    capture_physics_snapshot, Collider, MultibodyLink, PhysicsBackend, PhysicsCapability,
    PhysicsMaterial, PhysicsSnapshot, PhysicsWorldDesc, RaycastQuery, RevoluteJointDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_analytic::AnalyticBackend;
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const CONFORMANCE_REPORT_SCHEMA_VERSION: u16 = 1;
const TOLERANCE_REGISTRY_VERSION: u16 = 1;

const BACKEND_ANALYTIC: &str = "analytic";
const BACKEND_RAPIER: &str = "rapier";
const BACKEND_COMPARISON: &str = "analytic_vs_rapier";
const CASE_ANALYTIC_RIGID: &str = "analytic.rigid_body.free_fall";
const CASE_ANALYTIC_DETERMINISM: &str = "analytic.deterministic_step.repeat_snapshot";
const CASE_RAPIER_RIGID: &str = "rapier.rigid_body.free_fall";
const CASE_RAPIER_DETERMINISM: &str = "rapier.deterministic_step.repeat_snapshot";
const CASE_RAPIER_ARTICULATION: &str = "rapier.articulation.revolute_limit";
const CASE_RAPIER_CONTACT: &str = "rapier.contact_force.resting_impulse";
const CASE_RAPIER_RAYCAST: &str = "rapier.raycast_batch.ordered_hits";
const CASE_BACKEND_COMPARISON: &str = "analytic_vs_rapier.free_fall";

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
    Unitless,
}

impl MetricUnit {
    const fn symbol(self) -> &'static str {
        match self {
            Self::Metre => "m",
            Self::MetrePerSecond => "m/s",
            Self::Radian => "rad",
            Self::NewtonSecond => "N*s",
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
];

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct ToleranceReport {
    id: String,
    absolute: f64,
    relative: f64,
    allowed_error: f64,
    rationale: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
struct CaseReport {
    id: String,
    backend: String,
    capability: String,
    passed: bool,
    snapshot_hash: Option<u64>,
    metrics: Vec<MetricReport>,
    detail: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct BackendReport {
    name: String,
    advertised_capabilities: Vec<String>,
    covered_capabilities: Vec<String>,
    coverage_passed: bool,
}

/// Deterministic result of the complete backend capability catalog.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConformanceReport {
    schema_version: u16,
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

#[derive(Clone, Debug)]
struct FreeFallResult {
    position_y_m: f64,
    velocity_y_m_s: f64,
    snapshot: PhysicsSnapshot,
}

fn tolerance(id: &str) -> &'static ToleranceSpec {
    TOLERANCES
        .iter()
        .find(|tolerance| tolerance.id == id)
        .unwrap_or_else(|| panic!("unknown conformance tolerance {id}"))
}

/// Executes every deterministic and tolerance-bounded physics conformance case.
pub fn run_conformance() -> ConformanceReport {
    let analytic_capabilities = capability_names(AnalyticBackend::new().capabilities());
    let rapier_capabilities = capability_names(RapierBackend::new().capabilities());

    let analytic = run_free_fall(AnalyticBackend::new());
    let rapier = run_free_fall(RapierBackend::new());
    let mut cases = vec![
        result_case(
            CASE_ANALYTIC_RIGID,
            BACKEND_ANALYTIC,
            PhysicsCapability::RigidBody,
            analytic.as_ref().map(|result| {
                free_fall_case(
                    CASE_ANALYTIC_RIGID,
                    BACKEND_ANALYTIC,
                    PhysicsCapability::RigidBody,
                    result,
                    semi_implicit_free_fall_y(),
                    "analytic_free_fall_position_m_v1",
                )
            }),
        ),
        determinism_case::<AnalyticBackend>(
            CASE_ANALYTIC_DETERMINISM,
            BACKEND_ANALYTIC,
            AnalyticBackend::new,
        ),
        result_case(
            CASE_RAPIER_RIGID,
            BACKEND_RAPIER,
            PhysicsCapability::RigidBody,
            rapier.as_ref().map(|result| {
                free_fall_case(
                    CASE_RAPIER_RIGID,
                    BACKEND_RAPIER,
                    PhysicsCapability::RigidBody,
                    result,
                    continuous_free_fall_y(),
                    "rapier_free_fall_position_m_v1",
                )
            }),
        ),
        determinism_case::<RapierBackend>(
            CASE_RAPIER_DETERMINISM,
            BACKEND_RAPIER,
            RapierBackend::new,
        ),
        result_case(
            CASE_RAPIER_ARTICULATION,
            BACKEND_RAPIER,
            PhysicsCapability::Articulation,
            run_articulation_case(),
        ),
        result_case(
            CASE_RAPIER_CONTACT,
            BACKEND_RAPIER,
            PhysicsCapability::ContactForce,
            run_contact_case(),
        ),
        result_case(
            CASE_RAPIER_RAYCAST,
            BACKEND_RAPIER,
            PhysicsCapability::RaycastBatch,
            run_raycast_case(),
        ),
        comparison_case(analytic.as_ref(), rapier.as_ref()),
    ];
    cases.sort_by(|left, right| left.id.cmp(&right.id));

    let backends = vec![
        backend_report(BACKEND_ANALYTIC, analytic_capabilities, &cases),
        backend_report(BACKEND_RAPIER, rapier_capabilities, &cases),
    ];
    let all_passed = cases.iter().all(|case| case.passed)
        && backends.iter().all(|backend| backend.coverage_passed);
    ConformanceReport {
        schema_version: CONFORMANCE_REPORT_SCHEMA_VERSION,
        tolerance_registry_version: TOLERANCE_REGISTRY_VERSION,
        backends,
        cases,
        all_passed,
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
    let contacts = backend.contacts(physics_world)?.to_vec();
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
        capability: capability.as_str().to_string(),
        passed: metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(result.snapshot.stable_hash()),
        metrics,
        detail: "shared 60 Hz free-fall vector".to_string(),
    }
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
                capability: PhysicsCapability::DeterministicStep.as_str().to_string(),
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

fn run_articulation_case() -> anyhow::Result<CaseReport> {
    let mut backend = RapierBackend::new();
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
        rne_physics::JointMotor {
            velocity_rad_s: 2.0,
            gain: 1.0,
            stiffness: 0.0,
            target_position: 0.0,
            max_force: 50.0,
        },
    ));
    for _ in 0..180 {
        rne_physics_rapier::step_physics(&mut backend, &mut world, physics_world, fixed_dt())?;
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
    let joint_angle_abs_rad = backend
        .multibody_joint_position(physics_world, child)
        .context("reduced-coordinate joint position missing")?
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
    let contacts = backend.contacts(physics_world)?.to_vec();
    let snapshot = capture_physics_snapshot(&world, &contacts, 180, fixed_dt().ticks() * 180)?;
    Ok(CaseReport {
        id: CASE_RAPIER_ARTICULATION.to_string(),
        backend: BACKEND_RAPIER.to_string(),
        capability: PhysicsCapability::Articulation.as_str().to_string(),
        passed: metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(snapshot.stable_hash()),
        metrics,
        detail: "revolute motor pushes against a bounded joint limit".to_string(),
    })
}

fn run_contact_case() -> anyhow::Result<CaseReport> {
    let mut backend = RapierBackend::new();
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
        rne_physics_rapier::step_physics(&mut backend, &mut world, physics_world, fixed_dt())?;
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
    // Rapier currently interprets `RigidBody::mass_kg` as additional mass and
    // adds the cuboid's unit-density 0.5 m ^ 3 shape mass.
    let effective_mass_kg = mass_kg + 0.5_f64.powi(3);
    let expected = effective_mass_kg * GRAVITY_M_S2.abs() * fixed_dt().as_seconds().value();
    let metrics = vec![metric(
        "mean_normal_impulse_n_s",
        measured,
        expected,
        "resting_impulse_n_s_v1",
    )];
    let contacts = backend.contacts(physics_world)?.to_vec();
    let snapshot = capture_physics_snapshot(&world, &contacts, 180, fixed_dt().ticks() * 180)?;
    let pair_is_present = snapshot.contacts.iter().any(|contact| {
        contact.entity_a_index == ground.index().min(cube.index())
            && contact.entity_b_index == ground.index().max(cube.index())
            && contact.normal_impulse_n_s > 0.0
    });
    Ok(CaseReport {
        id: CASE_RAPIER_CONTACT.to_string(),
        backend: BACKEND_RAPIER.to_string(),
        capability: PhysicsCapability::ContactForce.as_str().to_string(),
        passed: pair_is_present && metrics.iter().all(|metric| metric.passed),
        snapshot_hash: Some(snapshot.stable_hash()),
        metrics,
        detail: format!(
            "{} of 60 settled steps reported the canonical load-bearing pair; effective_mass_kg={effective_mass_kg}",
            impulses.len(),
        ),
    })
}

fn run_raycast_case() -> anyhow::Result<CaseReport> {
    let mut backend = RapierBackend::new();
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
        id: CASE_RAPIER_RAYCAST.to_string(),
        backend: BACKEND_RAPIER.to_string(),
        capability: PhysicsCapability::RaycastBatch.as_str().to_string(),
        passed: shape_ok && ordering_ok && repeat_ok && metrics.iter().all(|metric| metric.passed),
        snapshot_hash: None,
        metrics,
        detail: format!(
            "query_shape_ok={shape_ok} ordering_ok={ordering_ok} repeat_ok={repeat_ok}"
        ),
    })
}

fn comparison_case(
    analytic: Result<&FreeFallResult, &anyhow::Error>,
    rapier: Result<&FreeFallResult, &anyhow::Error>,
) -> CaseReport {
    match (analytic, rapier) {
        (Ok(analytic), Ok(rapier)) => {
            let metrics = vec![
                metric(
                    "position_delta_m",
                    (analytic.position_y_m - rapier.position_y_m).abs(),
                    0.0,
                    "backend_free_fall_position_delta_m_v1",
                ),
                metric(
                    "velocity_delta_m_s",
                    (analytic.velocity_y_m_s - rapier.velocity_y_m_s).abs(),
                    0.0,
                    "backend_free_fall_velocity_delta_m_s_v1",
                ),
            ];
            CaseReport {
                id: CASE_BACKEND_COMPARISON.to_string(),
                backend: BACKEND_COMPARISON.to_string(),
                capability: PhysicsCapability::RigidBody.as_str().to_string(),
                passed: metrics.iter().all(|metric| metric.passed),
                snapshot_hash: None,
                metrics,
                detail: "same input vector; approximate solver comparison".to_string(),
            }
        }
        (Err(error), _) | (_, Err(error)) => failed_case(
            CASE_BACKEND_COMPARISON,
            BACKEND_COMPARISON,
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
        capability: capability.as_str().to_string(),
        passed: false,
        snapshot_hash: None,
        metrics: Vec::new(),
        detail,
    }
}

fn backend_report(
    name: &str,
    advertised_capabilities: Vec<String>,
    cases: &[CaseReport],
) -> BackendReport {
    let covered = cases
        .iter()
        .filter(|case| case.backend == name && case.passed)
        .map(|case| case.capability.clone())
        .collect::<BTreeSet<_>>();
    let covered_capabilities = advertised_capabilities
        .iter()
        .filter(|capability| covered.contains(*capability))
        .cloned()
        .collect::<Vec<_>>();
    let coverage_passed = covered_capabilities == advertised_capabilities;
    BackendReport {
        name: name.to_string(),
        advertised_capabilities,
        covered_capabilities,
        coverage_passed,
    }
}

fn capability_names(capabilities: &[PhysicsCapability]) -> Vec<String> {
    PhysicsCapability::ALL
        .iter()
        .filter(|capability| capabilities.contains(capability))
        .map(|capability| capability.as_str().to_string())
        .collect()
}

fn fixed_dt() -> SimDuration {
    SimDuration::from_hertz(Hertz::new(STEP_HZ))
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
            .all(|backend| backend.coverage_passed));
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
}
