//! Physics backend trait and world identifiers.

use crate::{ContactEvent, RaycastHit, RaycastQuery};
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Current schema version for backend capability manifests.
pub const PHYSICS_BACKEND_MANIFEST_SCHEMA_VERSION: u16 = 3;

/// Current schema version for aggregate physics conformance reports.
pub const PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION: u16 = 2;

/// Current version of the named, unit-bearing physics tolerance registry.
pub const PHYSICS_TOLERANCE_REGISTRY_VERSION: u16 = 2;

/// Identifier for a backend-owned physics world instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PhysicsWorldId(pub u32);

impl PhysicsWorldId {
    /// Default physics world identifier.
    pub const DEFAULT: Self = Self(0);
}

/// Initial configuration for a physics world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsWorldDesc {
    /// Gravity vector in meters per second squared.
    pub gravity_m_s2: Vec3,
    /// Constraint solver iterations per step. `0` uses the backend default; a higher
    /// value stabilizes stiff articulated chains (several jointed links) at extra cost.
    pub solver_iterations: usize,
}

impl Default for PhysicsWorldDesc {
    fn default() -> Self {
        Self {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            solver_iterations: 0,
        }
    }
}

/// Optional physics backend capabilities.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicsCapability {
    /// Supports rigid body simulation.
    RigidBody,
    /// Supports articulated bodies.
    Articulation,
    /// Supports GPU rigid body simulation.
    GpuRigidBody,
    /// Supports deterministic stepping.
    DeterministicStep,
    /// Supports soft bodies.
    SoftBody,
    /// Supports contact force reporting.
    ContactForce,
    /// Supports batched raycasts.
    RaycastBatch,
    /// Supports externally posed kinematic rigid bodies.
    KinematicBody,
    /// Retains backend-measured joint effort from the completed simulation step.
    JointEffortMeasurement,
}

/// Repeatability promise made by a physics backend manifest.
///
/// This classifies fresh executions on the same supported runtime. Cross-backend
/// and cross-platform comparisons remain governed by the conformance report's
/// named tolerance registry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicsBackendRepeatability {
    /// Canonical snapshots must compare exactly on the same runtime and platform.
    SameRuntimeExact,
    /// Numeric observables are compared only through named conformance tolerances.
    ToleranceBounded,
}

/// Versioned, backend-neutral declaration consumed by conformance runners.
///
/// Backend crates construct this value from stable identifiers and capability
/// declarations. Engine-specific handles and native types never enter the
/// manifest or the [`PhysicsBackend`] trait.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicsBackendManifest {
    /// Manifest schema version.
    pub schema_version: u16,
    /// Stable lowercase backend identifier, such as `analytic` or `rapier`.
    pub backend_id: String,
    /// Version of the RNE backend adapter.
    pub adapter_version: String,
    /// Stable identifier of the underlying dynamics engine.
    pub engine_id: String,
    /// Version of the underlying dynamics engine or algorithm contract.
    pub engine_version: String,
    /// Capabilities in [`PhysicsCapability::ALL`] order without duplicates.
    pub capabilities: Vec<PhysicsCapability>,
    /// Same-runtime repeatability promised by this backend.
    pub repeatability: PhysicsBackendRepeatability,
}

impl PhysicsBackendManifest {
    /// Creates and validates a backend manifest using the current schema.
    pub fn new(
        backend_id: impl Into<String>,
        adapter_version: impl Into<String>,
        engine_id: impl Into<String>,
        engine_version: impl Into<String>,
        capabilities: impl IntoIterator<Item = PhysicsCapability>,
        repeatability: PhysicsBackendRepeatability,
    ) -> Result<Self, PhysicsBackendManifestError> {
        let manifest = Self {
            schema_version: PHYSICS_BACKEND_MANIFEST_SCHEMA_VERSION,
            backend_id: backend_id.into(),
            adapter_version: adapter_version.into(),
            engine_id: engine_id.into(),
            engine_version: engine_version.into(),
            capabilities: capabilities.into_iter().collect(),
            repeatability,
        };
        manifest.validate()?;
        Ok(manifest)
    }

    /// Validates schema, identifiers, canonical capability order, and repeatability.
    pub fn validate(&self) -> Result<(), PhysicsBackendManifestError> {
        if self.schema_version != PHYSICS_BACKEND_MANIFEST_SCHEMA_VERSION {
            return Err(PhysicsBackendManifestError::UnsupportedSchemaVersion {
                expected: PHYSICS_BACKEND_MANIFEST_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        for (field, value) in [
            ("backend_id", self.backend_id.as_str()),
            ("adapter_version", self.adapter_version.as_str()),
            ("engine_id", self.engine_id.as_str()),
            ("engine_version", self.engine_version.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(PhysicsBackendManifestError::EmptyField { field });
            }
        }
        for (field, value) in [
            ("backend_id", self.backend_id.as_str()),
            ("engine_id", self.engine_id.as_str()),
        ] {
            if !is_stable_identifier(value) {
                return Err(PhysicsBackendManifestError::InvalidIdentifier { field });
            }
        }
        let canonical = PhysicsCapability::ALL
            .iter()
            .filter(|capability| self.capabilities.contains(capability))
            .copied()
            .collect::<Vec<_>>();
        if canonical != self.capabilities {
            return Err(PhysicsBackendManifestError::NonCanonicalCapabilities);
        }
        if self.repeatability == PhysicsBackendRepeatability::SameRuntimeExact
            && !self
                .capabilities
                .contains(&PhysicsCapability::DeterministicStep)
        {
            return Err(PhysicsBackendManifestError::ExactRepeatabilityWithoutDeterministicStep);
        }
        if self
            .capabilities
            .contains(&PhysicsCapability::KinematicBody)
            && !self.capabilities.contains(&PhysicsCapability::RigidBody)
        {
            return Err(PhysicsBackendManifestError::KinematicWithoutRigidBody);
        }
        if self
            .capabilities
            .contains(&PhysicsCapability::JointEffortMeasurement)
            && !self.capabilities.contains(&PhysicsCapability::Articulation)
        {
            return Err(PhysicsBackendManifestError::JointEffortWithoutArticulation);
        }
        Ok(())
    }
}

/// Invalid backend-manifest declaration.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum PhysicsBackendManifestError {
    /// The manifest schema is not supported by this engine version.
    #[error("unsupported physics backend manifest schema: expected {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Manifest schema understood by this engine.
        expected: u16,
        /// Manifest schema supplied by the backend.
        actual: u16,
    },
    /// A required stable identifier or version is empty.
    #[error("physics backend manifest field {field} must not be empty")]
    EmptyField {
        /// Name of the empty manifest field.
        field: &'static str,
    },
    /// A backend or engine identifier is not stable lowercase ASCII.
    #[error("physics backend manifest field {field} must match [a-z][a-z0-9_]*")]
    InvalidIdentifier {
        /// Name of the invalid manifest field.
        field: &'static str,
    },
    /// Capabilities are duplicated or not in the canonical engine order.
    #[error("physics backend manifest capabilities must be unique and canonically ordered")]
    NonCanonicalCapabilities,
    /// Exact repeatability was claimed without the corresponding capability.
    #[error("same-runtime exact repeatability requires deterministic_step capability")]
    ExactRepeatabilityWithoutDeterministicStep,
    /// Kinematic-body support refines the rigid-body capability.
    #[error("kinematic_body capability requires rigid_body capability")]
    KinematicWithoutRigidBody,
    /// Joint-effort measurement refines the articulation capability.
    #[error("joint_effort_measurement capability requires articulation capability")]
    JointEffortWithoutArticulation,
}

fn is_stable_identifier(value: &str) -> bool {
    let mut chars = value.chars();
    matches!(chars.next(), Some(first) if first.is_ascii_lowercase())
        && chars.all(|character| {
            character.is_ascii_lowercase() || character.is_ascii_digit() || character == '_'
        })
}

impl PhysicsCapability {
    /// Every capability known by this engine version in stable wire/report order.
    pub const ALL: [Self; 9] = [
        Self::RigidBody,
        Self::Articulation,
        Self::GpuRigidBody,
        Self::DeterministicStep,
        Self::SoftBody,
        Self::ContactForce,
        Self::RaycastBatch,
        Self::KinematicBody,
        Self::JointEffortMeasurement,
    ];

    /// Returns the stable lowercase identifier used by conformance reports.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::RigidBody => "rigid_body",
            Self::KinematicBody => "kinematic_body",
            Self::Articulation => "articulation",
            Self::GpuRigidBody => "gpu_rigid_body",
            Self::DeterministicStep => "deterministic_step",
            Self::SoftBody => "soft_body",
            Self::ContactForce => "contact_force",
            Self::RaycastBatch => "raycast_batch",
            Self::JointEffortMeasurement => "joint_effort_measurement",
        }
    }
}

/// Physics backend error type.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum PhysicsError {
    /// Requested physics world does not exist.
    #[error("physics world not found")]
    WorldNotFound,
    /// Backend failed to initialize.
    #[error("physics backend initialization failed")]
    InitializationFailed,
    /// The backend does not satisfy every required capability.
    #[error("physics backend lacks required capabilities: {missing:?}")]
    MissingCapabilities {
        /// Capabilities the backend does not provide.
        missing: Vec<PhysicsCapability>,
    },
    /// A joint actuation command is invalid for the target entity.
    #[error("invalid joint actuation on entity {entity_index}: {reason}")]
    InvalidActuation {
        /// Stable ECS entity index carrying the rejected command.
        entity_index: u32,
        /// Static validation reason shared by backend implementations.
        reason: &'static str,
    },
    /// Passive joint dynamics are invalid for the target entity.
    #[error("invalid passive joint dynamics on entity {entity_index}: {reason}")]
    InvalidPassiveDynamics {
        /// Stable ECS entity index carrying the rejected plant parameters.
        entity_index: u32,
        /// Static validation reason shared by backend implementations.
        reason: &'static str,
    },
    /// Exact rigid-body inertial properties are invalid.
    #[error("invalid rigid-body inertia on entity {entity_index}: {reason}")]
    InvalidInertia {
        /// Stable ECS entity index carrying the rejected properties.
        entity_index: u32,
        /// Static validation reason shared by backend implementations.
        reason: &'static str,
    },
}

/// Verifies a backend's declared capabilities satisfy every required one.
///
/// Returns [`PhysicsError::MissingCapabilities`] listing the capabilities the
/// backend does not provide. Order-insensitive: required capabilities are
/// deduplicated and compared as a set.
pub fn require_capabilities(
    available: &[PhysicsCapability],
    required: &[PhysicsCapability],
) -> Result<(), PhysicsError> {
    let mut missing = Vec::new();
    for capability in required {
        if !available.contains(capability) && !missing.contains(capability) {
            missing.push(*capability);
        }
    }
    if missing.is_empty() {
        Ok(())
    } else {
        missing.sort_unstable();
        Err(PhysicsError::MissingCapabilities { missing })
    }
}

/// Backend-agnostic physics simulation interface.
pub trait PhysicsBackend: Send + Sync + 'static {
    /// Opaque rigid body handle type.
    type BodyHandle: Copy + Send + Sync + std::fmt::Debug;
    /// Opaque collider handle type.
    type ColliderHandle: Copy + Send + Sync + std::fmt::Debug;

    /// Creates a new physics world and returns its identifier.
    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError>;

    /// Synchronizes ECS state into the physics world.
    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError>;

    /// Advances the physics simulation by one fixed step.
    fn step(&mut self, physics_world: PhysicsWorldId, dt: SimDuration) -> Result<(), PhysicsError>;

    /// Synchronizes physics state back into ECS transforms.
    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError>;

    /// Executes a raycast query.
    ///
    /// Implementations return every hit ordered by increasing distance, with
    /// stable entity order used to break equal-distance ties.
    fn raycast(
        &self,
        physics_world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError>;

    /// Executes raycast queries in caller-provided order.
    ///
    /// The outer result preserves query order. Each inner hit list follows the
    /// same distance/entity ordering contract as [`Self::raycast`]. Backends
    /// advertising [`PhysicsCapability::RaycastBatch`] must pass the conformance
    /// vector for this method.
    fn raycast_batch(
        &self,
        physics_world: PhysicsWorldId,
        queries: &[RaycastQuery],
    ) -> Result<Vec<Vec<RaycastHit>>, PhysicsError> {
        queries
            .iter()
            .copied()
            .map(|query| self.raycast(physics_world, query))
            .collect()
    }

    /// Returns contact events from the last simulation step.
    fn contacts(&self, physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError>;

    /// Returns supported capabilities for this backend.
    fn capabilities(&self) -> &[PhysicsCapability];
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::{Collider, RigidBody};
    use rne_ecs::spawn_named;
    use rne_world::Transform3;

    struct MockBackend {
        worlds: Vec<PhysicsWorldDesc>,
        contacts: Vec<ContactEvent>,
    }

    impl MockBackend {
        fn new() -> Self {
            Self {
                worlds: Vec::new(),
                contacts: Vec::new(),
            }
        }
    }

    impl PhysicsBackend for MockBackend {
        type BodyHandle = u32;
        type ColliderHandle = u32;

        fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
            self.worlds.push(desc);
            Ok(PhysicsWorldId(self.worlds.len() as u32 - 1))
        }

        fn sync_from_ecs(
            &mut self,
            world: &mut World,
            _physics_world: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            let _count = world
                .query::<(&RigidBody, &Collider, &Transform3)>()
                .iter(world)
                .count();
            Ok(())
        }

        fn step(
            &mut self,
            _physics_world: PhysicsWorldId,
            _dt: SimDuration,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn sync_to_ecs(
            &mut self,
            _world: &mut World,
            _physics_world: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn raycast(
            &self,
            _physics_world: PhysicsWorldId,
            _query: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            Ok(Vec::new())
        }

        fn contacts(
            &self,
            _physics_world: PhysicsWorldId,
        ) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&self.contacts)
        }

        fn capabilities(&self) -> &[PhysicsCapability] {
            &[PhysicsCapability::RigidBody]
        }
    }

    #[test]
    fn require_capabilities_lists_missing_features() {
        let available = [
            PhysicsCapability::RigidBody,
            PhysicsCapability::Articulation,
        ];
        require_capabilities(&available, &[PhysicsCapability::Articulation])
            .expect("present capability is accepted");

        let error = require_capabilities(
            &available,
            &[
                PhysicsCapability::SoftBody,
                PhysicsCapability::Articulation,
                PhysicsCapability::GpuRigidBody,
            ],
        )
        .expect_err("missing capabilities are rejected");
        assert!(matches!(
            error,
            PhysicsError::MissingCapabilities { missing }
                if missing == vec![PhysicsCapability::GpuRigidBody, PhysicsCapability::SoftBody]
        ));
    }

    #[test]
    fn backend_manifest_round_trips_with_canonical_capabilities() {
        let manifest = PhysicsBackendManifest::new(
            "analytic",
            "0.1.0",
            "rne_analytic",
            "1",
            [
                PhysicsCapability::RigidBody,
                PhysicsCapability::DeterministicStep,
            ],
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect("valid manifest");

        let json = serde_json::to_string(&manifest).expect("serialize manifest");
        let decoded: PhysicsBackendManifest =
            serde_json::from_str(&json).expect("deserialize manifest");
        assert_eq!(decoded, manifest);
        decoded
            .validate()
            .expect("round-tripped manifest validates");
    }

    #[test]
    fn backend_manifest_rejects_order_duplicates_and_false_exact_claims() {
        let unordered = PhysicsBackendManifest::new(
            "rapier",
            "0.1.0",
            "rapier3d",
            "0.22",
            [
                PhysicsCapability::DeterministicStep,
                PhysicsCapability::RigidBody,
            ],
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect_err("non-canonical order must fail");
        assert_eq!(
            unordered,
            PhysicsBackendManifestError::NonCanonicalCapabilities
        );

        let duplicate = PhysicsBackendManifest::new(
            "rapier",
            "0.1.0",
            "rapier3d",
            "0.22",
            [PhysicsCapability::RigidBody, PhysicsCapability::RigidBody],
            PhysicsBackendRepeatability::ToleranceBounded,
        )
        .expect_err("duplicate capability must fail");
        assert_eq!(
            duplicate,
            PhysicsBackendManifestError::NonCanonicalCapabilities
        );

        let false_exact = PhysicsBackendManifest::new(
            "mujoco",
            "0.1.0",
            "mujoco",
            "3.9.0",
            [PhysicsCapability::RigidBody],
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect_err("exact claim needs deterministic capability");
        assert_eq!(
            false_exact,
            PhysicsBackendManifestError::ExactRepeatabilityWithoutDeterministicStep
        );

        let invalid_id = PhysicsBackendManifest::new(
            "MuJoCo",
            "0.1.0",
            "mujoco",
            "3.9.0",
            [PhysicsCapability::RigidBody],
            PhysicsBackendRepeatability::ToleranceBounded,
        )
        .expect_err("wire identifiers must be stable lowercase ASCII");
        assert_eq!(
            invalid_id,
            PhysicsBackendManifestError::InvalidIdentifier {
                field: "backend_id"
            }
        );

        let orphan_kinematic = PhysicsBackendManifest::new(
            "fixture",
            "0.1.0",
            "fixture",
            "1",
            [PhysicsCapability::KinematicBody],
            PhysicsBackendRepeatability::ToleranceBounded,
        )
        .expect_err("kinematic support refines rigid-body support");
        assert_eq!(
            orphan_kinematic,
            PhysicsBackendManifestError::KinematicWithoutRigidBody
        );

        let orphan_joint_effort = PhysicsBackendManifest::new(
            "fixture",
            "0.1.0",
            "fixture",
            "1",
            [PhysicsCapability::JointEffortMeasurement],
            PhysicsBackendRepeatability::ToleranceBounded,
        )
        .expect_err("joint-effort measurement refines articulation support");
        assert_eq!(
            orphan_joint_effort,
            PhysicsBackendManifestError::JointEffortWithoutArticulation
        );
    }

    #[test]
    fn mock_backend_registers_world_and_syncs_entities() {
        let mut backend = MockBackend::new();
        let world_id = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("world");
        assert_eq!(world_id, PhysicsWorldId(0));

        let mut world = World::new();
        let entity = spawn_named(&mut world, "cube");
        world.entity_mut(entity).insert((
            RigidBody::default(),
            Collider::default(),
            Transform3::default(),
        ));

        backend
            .sync_from_ecs(&mut world, world_id)
            .expect("sync from ecs");
        backend
            .step(
                world_id,
                SimDuration::from_hertz(rne_math::Hertz::new(60.0)),
            )
            .expect("step");
        backend
            .sync_to_ecs(&mut world, world_id)
            .expect("sync to ecs");
    }
}
