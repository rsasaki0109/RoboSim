//! Feature-gated MuJoCo backend implementation.

use crate::compiler::{
    compile_rigid_body_model, legacy_motor_gains, BodyBinding, BodyTopology, CompileError,
    CompiledRigidBodyModel, JointBinding, JointDynamics,
};
use crate::EXPECTED_MUJOCO_VERSION_PREFIX;
use mujoco_rs::prelude::{MjData, MjModel, MjtObj};
use rne_core::SimDuration;
use rne_ecs::{Entity, Parent, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    ColliderShape, ContactEvent, JointActuation, JointMotor, JointState, PhysicsBackend,
    PhysicsCapability, PhysicsError, PhysicsWorldDesc, PhysicsWorldId, RaycastHit, RaycastQuery,
    RigidBody, RigidBodyType,
};
use rne_world::{world_transform_of, Transform3};
use std::collections::{BTreeMap, HashMap};
use std::sync::Mutex;
use thiserror::Error;

const FREE_FALL_BODY_NAME: &str = "rne_free_fall_body";
const FREE_FALL_JOINT_NAME: &str = "rne_free_fall_joint";
const EXPECTED_FREE_JOINT_QPOS_LEN: usize = 7;
const EXPECTED_FREE_JOINT_QVEL_LEN: usize = 6;
const CAPABILITIES: &[PhysicsCapability] = &[
    PhysicsCapability::RigidBody,
    PhysicsCapability::Articulation,
    PhysicsCapability::ContactForce,
    PhysicsCapability::RaycastBatch,
];

/// Errors specific to the optional MuJoCo adapter.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum MuJoCoError {
    /// The caller supplied a value that cannot be represented by MuJoCo.
    #[error("invalid MuJoCo input: {0}")]
    InvalidInput(String),
    /// The loaded native library is not on the ABI line used by this crate.
    #[error("incompatible MuJoCo runtime: expected {expected}x, found {found}")]
    RuntimeVersionMismatch {
        /// Expected runtime version prefix.
        expected: &'static str,
        /// Runtime version reported by the native library.
        found: String,
    },
    /// MuJoCo rejected the bounded MJCF fixture.
    #[error("MuJoCo failed to load the fixture: {0}")]
    ModelLoad(String),
    /// MuJoCo could not allocate its per-world state.
    #[error("MuJoCo failed to allocate world data")]
    DataAllocation,
    /// The model does not match the supported free-joint sphere fixture.
    #[error("unsupported MuJoCo fixture: {0}")]
    UnsupportedFixture(String),
    /// The ECS world requires a capability this backend does not advertise.
    #[error("entity {entity_index} requires unsupported MuJoCo capability {capability:?}")]
    MissingCapability {
        /// Capability required by the rejected ECS entity.
        capability: PhysicsCapability,
        /// Stable ECS entity index that requires the capability.
        entity_index: u32,
    },
    /// The fixed topology changed after the native model was compiled.
    #[error("MuJoCo world topology changed after step 0: {detail}")]
    TopologyChanged {
        /// First stable topology difference found during synchronization.
        detail: String,
    },
    /// A unit-explicit joint actuation command is invalid.
    #[error("invalid MuJoCo joint actuation on entity {entity_index}: {reason}")]
    InvalidActuation {
        /// Stable ECS entity index carrying the rejected command.
        entity_index: u32,
        /// Static validation reason.
        reason: &'static str,
    },
    /// Exact rigid-body inertial properties are invalid.
    #[error("invalid MuJoCo rigid-body inertia on entity {entity_index}: {reason}")]
    InvalidInertia {
        /// Stable ECS entity index carrying the rejected properties.
        entity_index: u32,
        /// Static validation reason.
        reason: &'static str,
    },
    /// A fixed-step duration did not match the model timestep.
    #[error("MuJoCo timestep mismatch: expected {expected_s:.12} s, got {actual_s:.12} s")]
    TimestepMismatch {
        /// Timestep compiled into the fixture.
        expected_s: f64,
        /// Timestep requested by the RNE scheduler.
        actual_s: f64,
    },
    /// MuJoCo produced a non-finite state value.
    #[error("MuJoCo produced a non-finite value in {0}")]
    NonFiniteState(&'static str),
}

/// Opaque body handle owned by the MuJoCo adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MuJoCoBodyHandle(pub(crate) u32);

/// Opaque collider handle owned by the MuJoCo adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MuJoCoColliderHandle(pub(crate) u32);

/// MuJoCo-backed rigid-body adapter with backend-private ECS-to-MJCF compilation.
///
/// MuJoCo model and data types remain private implementation details. Dynamic
/// bodies use free joints, fixed bodies are welded into the compiled world, and
/// state crosses the backend boundary through RNE transforms and velocities.
/// Contact reporting includes canonical pair aggregation and sensor overlaps;
/// raycasts advertise `raycast_batch` via repeated native `mj_ray` queries.
#[derive(Debug)]
pub struct MuJoCoBackend {
    model_source: ModelSource,
    worlds: HashMap<PhysicsWorldId, MuJoCoWorld>,
    next_world_id: u32,
}

#[derive(Clone, Debug)]
enum ModelSource {
    EcsCompiler { timestep_s: f64 },
    CallerMjcf(String),
}

#[derive(Debug)]
struct MuJoCoWorld {
    /// Interior mutability lets `&self` raycasts use MuJoCo's `&mut MjData` API
    /// while keeping [`PhysicsBackend`]'s `Sync` bound.
    data: Mutex<Option<MjData<Box<MjModel>>>>,
    desc: PhysicsWorldDesc,
    bindings: Vec<BodyBinding>,
    topology: Vec<BodyTopology>,
    joint_dynamics: Vec<JointDynamics>,
    caller_mjcf: bool,
    timestep_s: f64,
    geom_entities: Vec<Option<Entity>>,
    sensor_geoms: Vec<bool>,
    contacts: Vec<ContactEvent>,
}

impl MuJoCoWorld {
    fn lock_data(&self) -> std::sync::MutexGuard<'_, Option<MjData<Box<MjModel>>>> {
        self.data
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

#[derive(Clone, Copy, Debug)]
struct ContactAccumulator {
    entity_a: Entity,
    entity_b: Entity,
    weighted_normal: Vec3,
    fallback_normal: Vec3,
    impulse_n_s: f64,
}

impl MuJoCoBackend {
    /// Returns the versioned conformance manifest without loading the native runtime.
    pub fn manifest() -> rne_physics::PhysicsBackendManifest {
        crate::backend_manifest()
    }

    /// Creates a backend that compiles rigid bodies from ECS before step 0.
    ///
    /// The fixed timestep is explicit because MuJoCo compiles it into each
    /// native model. Adding/removing bodies or changing fixed geometry after
    /// the first synchronization is rejected as a topology change.
    pub fn new(fixed_delta: SimDuration) -> Result<Self, MuJoCoError> {
        validate_runtime_version()?;
        let timestep_s = fixed_delta.as_seconds().value();
        if !timestep_s.is_finite() || timestep_s <= 0.0 {
            return Err(MuJoCoError::InvalidInput(
                "fixed timestep must be finite and positive".to_owned(),
            ));
        }
        Ok(Self {
            model_source: ModelSource::EcsCompiler { timestep_s },
            worlds: HashMap::new(),
            next_world_id: 0,
        })
    }

    /// Creates a backend from a bounded, caller-owned MJCF fixture.
    ///
    /// The native MuJoCo runtime is checked immediately.  The fixture must
    /// contain body `rne_free_fall_body` and joint `rne_free_fall_joint`, and
    /// must compile to the seven-coordinate/six-velocity free joint expected by
    /// this spike.
    pub fn from_mjcf(mjcf: impl Into<String>) -> Result<Self, MuJoCoError> {
        let mjcf = mjcf.into();
        if mjcf.contains('\0') {
            return Err(MuJoCoError::InvalidInput(
                "MJCF contains an interior NUL byte".to_owned(),
            ));
        }
        validate_runtime_version()?;
        if mjcf.trim().is_empty() {
            return Err(MuJoCoError::InvalidInput("MJCF is empty".to_owned()));
        }
        Ok(Self {
            model_source: ModelSource::CallerMjcf(mjcf),
            worlds: HashMap::new(),
            next_world_id: 0,
        })
    }

    /// Validates the ECS topology without creating a native MuJoCo model.
    ///
    /// This is the fail-fast capability boundary used before the first step.
    /// It reports unsupported topology before native model creation and keeps
    /// backend-native model types private.
    pub fn preflight_world(&self, world: &World) -> Result<(), MuJoCoError> {
        match &self.model_source {
            ModelSource::EcsCompiler { timestep_s } => {
                compile_rigid_body_model(world, PhysicsWorldDesc::default(), *timestep_s)
                    .map(|_| ())
                    .map_err(map_compile_error)
            }
            ModelSource::CallerMjcf(_) => validate_caller_fixture_world(world).map(|_| ()),
        }
    }

    /// Returns the native MuJoCo version after checking the supported ABI line.
    pub fn runtime_version() -> Result<&'static str, MuJoCoError> {
        validate_runtime_version()?;
        Ok(mujoco_rs::mujoco_version())
    }

    fn world(&self, id: PhysicsWorldId) -> Result<&MuJoCoWorld, PhysicsError> {
        self.worlds.get(&id).ok_or(PhysicsError::WorldNotFound)
    }

    fn world_mut(&mut self, id: PhysicsWorldId) -> Result<&mut MuJoCoWorld, PhysicsError> {
        self.worlds.get_mut(&id).ok_or(PhysicsError::WorldNotFound)
    }

    fn map_error(error: MuJoCoError) -> PhysicsError {
        match error {
            MuJoCoError::MissingCapability { capability, .. } => {
                PhysicsError::MissingCapabilities {
                    missing: vec![capability],
                }
            }
            MuJoCoError::InvalidActuation {
                entity_index,
                reason,
            } => PhysicsError::InvalidActuation {
                entity_index,
                reason,
            },
            MuJoCoError::InvalidInertia {
                entity_index,
                reason,
            } => PhysicsError::InvalidInertia {
                entity_index,
                reason,
            },
            _ => PhysicsError::InitializationFailed,
        }
    }
}

fn map_compile_error(error: CompileError) -> MuJoCoError {
    match error {
        CompileError::MissingCapability {
            capability,
            entity_index,
        } => MuJoCoError::MissingCapability {
            capability,
            entity_index,
        },
        CompileError::InvalidActuation {
            entity_index,
            reason,
        } => MuJoCoError::InvalidActuation {
            entity_index,
            reason,
        },
        CompileError::InvalidInertia {
            entity_index,
            reason,
        } => MuJoCoError::InvalidInertia {
            entity_index,
            reason,
        },
        other => MuJoCoError::UnsupportedFixture(other.to_string()),
    }
}

fn validate_runtime_version() -> Result<(), MuJoCoError> {
    let found = mujoco_rs::mujoco_version();
    if found.starts_with(EXPECTED_MUJOCO_VERSION_PREFIX) {
        Ok(())
    } else {
        Err(MuJoCoError::RuntimeVersionMismatch {
            expected: EXPECTED_MUJOCO_VERSION_PREFIX,
            found: found.to_owned(),
        })
    }
}

fn finite_vec3(value: Vec3, name: &'static str) -> Result<(), MuJoCoError> {
    if value.x.is_finite() && value.y.is_finite() && value.z.is_finite() {
        Ok(())
    } else {
        Err(MuJoCoError::NonFiniteState(name))
    }
}

fn finite_quat(value: Quat, name: &'static str) -> Result<(), MuJoCoError> {
    if value.x.is_finite() && value.y.is_finite() && value.z.is_finite() && value.w.is_finite() {
        Ok(())
    } else {
        Err(MuJoCoError::NonFiniteState(name))
    }
}

fn require_free_fall_model(model: &MjModel) -> Result<(), MuJoCoError> {
    if model
        .name_to_id(MjtObj::mjOBJ_BODY, FREE_FALL_BODY_NAME)
        .is_none()
    {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "missing body {FREE_FALL_BODY_NAME}"
        )));
    }
    if model
        .name_to_id(MjtObj::mjOBJ_JOINT, FREE_FALL_JOINT_NAME)
        .is_none()
    {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "missing joint {FREE_FALL_JOINT_NAME}"
        )));
    }
    if model.nq() as usize != EXPECTED_FREE_JOINT_QPOS_LEN
        || model.nv() as usize != EXPECTED_FREE_JOINT_QVEL_LEN
    {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "free joint dimensions must be nq={EXPECTED_FREE_JOINT_QPOS_LEN}, nv={EXPECTED_FREE_JOINT_QVEL_LEN}"
        )));
    }
    if !model.opt().timestep.is_finite() || model.opt().timestep <= 0.0 {
        return Err(MuJoCoError::UnsupportedFixture(
            "fixture timestep must be finite and positive".to_owned(),
        ));
    }
    Ok(())
}

fn validate_caller_fixture_world(world: &World) -> Result<CompiledRigidBodyModel, MuJoCoError> {
    let mut compiled = compile_rigid_body_model(world, PhysicsWorldDesc::default(), 1.0)
        .map_err(map_compile_error)?;
    if compiled.bindings.len() != 1 {
        return Err(MuJoCoError::UnsupportedFixture(
            "caller MJCF accepts exactly one ECS body".to_owned(),
        ));
    }
    let entity = compiled.bindings[0].entity;
    let rigid_body = world
        .get::<RigidBody>(entity)
        .ok_or_else(|| MuJoCoError::UnsupportedFixture("rigid body disappeared".to_owned()))?;
    let collider = world
        .get::<rne_physics::Collider>(entity)
        .ok_or_else(|| MuJoCoError::UnsupportedFixture("collider disappeared".to_owned()))?;
    if rigid_body.body_type != RigidBodyType::Dynamic
        || !matches!(collider.shape, ColliderShape::Sphere { .. })
    {
        return Err(MuJoCoError::UnsupportedFixture(
            "caller MJCF requires one dynamic sphere".to_owned(),
        ));
    }
    compiled.bindings[0].body_name = FREE_FALL_BODY_NAME.to_owned();
    compiled.bindings[0].joint = JointBinding::Free {
        joint_name: FREE_FALL_JOINT_NAME.to_owned(),
    };
    Ok(compiled)
}

fn require_compiled_model(model: &MjModel, bindings: &[BodyBinding]) -> Result<(), MuJoCoError> {
    let (expected_nq, expected_nv) = bindings.iter().fold((0, 0), |counts, binding| {
        let dimensions = match binding.joint {
            JointBinding::Free { .. } => {
                (EXPECTED_FREE_JOINT_QPOS_LEN, EXPECTED_FREE_JOINT_QVEL_LEN)
            }
            JointBinding::Revolute { .. } | JointBinding::Prismatic { .. } => (1, 1),
            JointBinding::Fixed => (0, 0),
        };
        (counts.0 + dimensions.0, counts.1 + dimensions.1)
    });
    if model.nq() as usize != expected_nq || model.nv() as usize != expected_nv {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "compiled joint dimensions must be nq={expected_nq}, nv={expected_nv}"
        )));
    }
    for binding in bindings {
        if model
            .name_to_id(MjtObj::mjOBJ_BODY, &binding.body_name)
            .is_none()
        {
            return Err(MuJoCoError::UnsupportedFixture(format!(
                "compiled model is missing body {}",
                binding.body_name
            )));
        }
        if let Some(name) = binding.joint.joint_name() {
            if model.name_to_id(MjtObj::mjOBJ_JOINT, name).is_none() {
                return Err(MuJoCoError::UnsupportedFixture(format!(
                    "compiled model is missing joint {name}"
                )));
            }
        }
        let actuator_name = match &binding.joint {
            JointBinding::Revolute { actuator_name, .. }
            | JointBinding::Prismatic { actuator_name, .. } => Some(actuator_name.as_str()),
            JointBinding::Free { .. } | JointBinding::Fixed => None,
        };
        if actuator_name
            .is_some_and(|name| model.name_to_id(MjtObj::mjOBJ_ACTUATOR, name).is_none())
        {
            return Err(MuJoCoError::UnsupportedFixture(format!(
                "compiled model is missing actuator {}",
                actuator_name.expect("checked Some")
            )));
        }
    }
    Ok(())
}

fn geometry_bindings(
    data: &MjData<Box<MjModel>>,
    bindings: &[BodyBinding],
    world: &World,
) -> Result<(Vec<Option<Entity>>, Vec<bool>), MuJoCoError> {
    let model = data.model();
    let mut body_entities = HashMap::new();
    for binding in bindings {
        let body_id = model
            .name_to_id(MjtObj::mjOBJ_BODY, &binding.body_name)
            .ok_or_else(|| {
                MuJoCoError::UnsupportedFixture(format!(
                    "compiled model is missing body {}",
                    binding.body_name
                ))
            })?;
        let sensor = world
            .get::<rne_physics::Collider>(binding.entity)
            .is_some_and(|collider| collider.sensor);
        body_entities.insert(body_id, (binding.entity, sensor));
    }

    let mut geom_entities = Vec::with_capacity(model.geom_bodyid().len());
    let mut sensor_geoms = Vec::with_capacity(model.geom_bodyid().len());
    for body_id in model.geom_bodyid() {
        let binding = usize::try_from(*body_id)
            .ok()
            .and_then(|body_id| body_entities.get(&body_id));
        geom_entities.push(binding.map(|(entity, _)| *entity));
        sensor_geoms.push(binding.is_some_and(|(_, sensor)| *sensor));
    }
    Ok((geom_entities, sensor_geoms))
}

fn collect_contact_events(
    data: &mut MjData<Box<MjModel>>,
    geom_entities: &[Option<Entity>],
    sensor_geoms: &[bool],
    timestep_s: f64,
) -> Result<Vec<ContactEvent>, MuJoCoError> {
    let mut pairs = BTreeMap::<(u32, u32), ContactAccumulator>::new();
    let contact_count = data.contact().len();
    for contact_id in 0..contact_count {
        let contact = &data.contact()[contact_id];
        let Some(entity_1) = usize::try_from(contact.geom1)
            .ok()
            .and_then(|index| geom_entities.get(index))
            .copied()
            .flatten()
        else {
            continue;
        };
        let Some(entity_2) = usize::try_from(contact.geom2)
            .ok()
            .and_then(|index| geom_entities.get(index))
            .copied()
            .flatten()
        else {
            continue;
        };
        if entity_1 == entity_2 {
            continue;
        }

        let native_normal = Vec3::from_slice(&contact.frame[..3]);
        let normal_force_n = data.contact_force(contact_id)[0];
        if !native_normal.is_finite() || !normal_force_n.is_finite() {
            return Err(MuJoCoError::NonFiniteState("contact evidence"));
        }
        let normal_length_squared = native_normal.length_squared();
        if normal_length_squared <= 1.0e-24 {
            return Err(MuJoCoError::UnsupportedFixture(
                "MuJoCo produced a zero contact normal".to_owned(),
            ));
        }
        let native_normal = native_normal / normal_length_squared.sqrt();
        let impulse_n_s = normal_force_n.max(0.0) * timestep_s;
        if !impulse_n_s.is_finite() {
            return Err(MuJoCoError::NonFiniteState("contact impulse"));
        }
        let (entity_a, entity_b, normal) = if entity_1.index() <= entity_2.index() {
            (entity_1, entity_2, native_normal)
        } else {
            (entity_2, entity_1, -native_normal)
        };
        let accumulator =
            pairs
                .entry((entity_a.index(), entity_b.index()))
                .or_insert(ContactAccumulator {
                    entity_a,
                    entity_b,
                    weighted_normal: Vec3::ZERO,
                    fallback_normal: Vec3::ZERO,
                    impulse_n_s: 0.0,
                });
        accumulator.weighted_normal += normal * impulse_n_s;
        accumulator.fallback_normal += normal;
        accumulator.impulse_n_s += impulse_n_s;
    }

    if geom_entities.len() != sensor_geoms.len() {
        return Err(MuJoCoError::UnsupportedFixture(
            "geometry binding lengths differ".to_owned(),
        ));
    }
    for geom_a in 0..geom_entities.len() {
        for geom_b in (geom_a + 1)..geom_entities.len() {
            if !(sensor_geoms[geom_a] || sensor_geoms[geom_b]) {
                continue;
            }
            let (Some(entity_1), Some(entity_2)) = (geom_entities[geom_a], geom_entities[geom_b])
            else {
                continue;
            };
            if entity_1 == entity_2 {
                continue;
            }
            let distance_m = data.geom_distance(geom_a, geom_b, 0.0, None);
            if !distance_m.is_finite() {
                return Err(MuJoCoError::NonFiniteState("sensor distance"));
            }
            if distance_m > 0.0 {
                continue;
            }
            let (entity_a, entity_b) = if entity_1.index() <= entity_2.index() {
                (entity_1, entity_2)
            } else {
                (entity_2, entity_1)
            };
            pairs
                .entry((entity_a.index(), entity_b.index()))
                .or_insert(ContactAccumulator {
                    entity_a,
                    entity_b,
                    weighted_normal: Vec3::ZERO,
                    fallback_normal: Vec3::ZERO,
                    impulse_n_s: 0.0,
                });
        }
    }

    pairs
        .into_values()
        .map(|pair| {
            if pair.impulse_n_s > f32::MAX as f64 {
                return Err(MuJoCoError::NonFiniteState("contact impulse"));
            }
            let weighted_length_squared = pair.weighted_normal.length_squared();
            let fallback_length_squared = pair.fallback_normal.length_squared();
            let normal = if weighted_length_squared > 1.0e-24 {
                pair.weighted_normal / weighted_length_squared.sqrt()
            } else if fallback_length_squared > 1.0e-24 {
                pair.fallback_normal / fallback_length_squared.sqrt()
            } else {
                Vec3::ZERO
            };
            Ok(ContactEvent {
                entity_a: pair.entity_a,
                entity_b: pair.entity_b,
                normal,
                impulse: pair.impulse_n_s as f32,
            })
        })
        .collect()
}

fn sync_from_ecs_state(
    data: &mut MjData<Box<MjModel>>,
    bindings: &[BodyBinding],
    world: &World,
) -> Result<(), MuJoCoError> {
    for binding in bindings {
        match &binding.joint {
            JointBinding::Free { joint_name } => {
                sync_free_joint_from_ecs(data, binding.entity, joint_name, world)?;
            }
            JointBinding::Revolute {
                joint_name,
                actuator_name,
            } => sync_scalar_joint_from_ecs(
                data,
                binding.entity,
                joint_name,
                actuator_name,
                true,
                world,
            )?,
            JointBinding::Prismatic {
                joint_name,
                actuator_name,
            } => sync_scalar_joint_from_ecs(
                data,
                binding.entity,
                joint_name,
                actuator_name,
                false,
                world,
            )?,
            JointBinding::Fixed => {}
        }
    }
    data.forward();
    Ok(())
}

fn sync_free_joint_from_ecs(
    data: &mut MjData<Box<MjModel>>,
    entity: rne_ecs::Entity,
    joint_name: &str,
    world: &World,
) -> Result<(), MuJoCoError> {
    let rigid_body = world.get::<RigidBody>(entity).ok_or_else(|| {
        MuJoCoError::UnsupportedFixture("rigid body disappeared during sync".to_owned())
    })?;
    let transform = world.get::<Transform3>(entity).ok_or_else(|| {
        MuJoCoError::UnsupportedFixture("transform disappeared during sync".to_owned())
    })?;
    finite_vec3(transform.translation, "position")?;
    finite_quat(transform.rotation, "rotation")?;
    finite_vec3(rigid_body.linear_velocity_m_s, "linear velocity")?;
    finite_vec3(rigid_body.angular_velocity_rad_s, "angular velocity")?;

    let joint = data.joint(joint_name).ok_or_else(|| {
        MuJoCoError::UnsupportedFixture(format!("missing compiled joint {joint_name}"))
    })?;
    let mut joint_view = joint.view_mut(data);
    if joint_view.qpos.len() != EXPECTED_FREE_JOINT_QPOS_LEN
        || joint_view.qvel.len() != EXPECTED_FREE_JOINT_QVEL_LEN
    {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "joint {joint_name} is not a free joint"
        )));
    }
    joint_view.qpos[..3].copy_from_slice(&[
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]);
    joint_view.qpos[3..7].copy_from_slice(&[
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
    ]);
    joint_view.qvel[..3].copy_from_slice(&[
        rigid_body.linear_velocity_m_s.x,
        rigid_body.linear_velocity_m_s.y,
        rigid_body.linear_velocity_m_s.z,
    ]);
    joint_view.qvel[3..6].copy_from_slice(&[
        rigid_body.angular_velocity_rad_s.x,
        rigid_body.angular_velocity_rad_s.y,
        rigid_body.angular_velocity_rad_s.z,
    ]);
    Ok(())
}

fn sync_scalar_joint_from_ecs(
    data: &mut MjData<Box<MjModel>>,
    entity: rne_ecs::Entity,
    joint_name: &str,
    actuator_name: &str,
    revolute: bool,
    world: &World,
) -> Result<(), MuJoCoError> {
    let joint = data.joint(joint_name).ok_or_else(|| {
        MuJoCoError::UnsupportedFixture(format!("missing compiled joint {joint_name}"))
    })?;
    let initial_state = world.get::<JointState>(entity).copied();
    if let Some(state) = initial_state {
        let (position, velocity) = match (revolute, state) {
            (
                true,
                JointState::Revolute {
                    position_rad,
                    velocity_rad_s,
                },
            ) => (position_rad, velocity_rad_s),
            (
                false,
                JointState::Prismatic {
                    position_m,
                    velocity_m_s,
                },
            ) => (position_m, velocity_m_s),
            _ => {
                return Err(MuJoCoError::InvalidActuation {
                    entity_index: entity.index(),
                    reason: "JointState kind does not match joint",
                });
            }
        };
        if !position.is_finite() || !velocity.is_finite() {
            return Err(MuJoCoError::InvalidActuation {
                entity_index: entity.index(),
                reason: "JointState is non-finite",
            });
        }
        let mut view = joint.view_mut(data);
        if view.qpos.len() != 1 || view.qvel.len() != 1 {
            return Err(MuJoCoError::UnsupportedFixture(format!(
                "joint {joint_name} is not scalar"
            )));
        }
        view.qpos[0] = position;
        view.qvel[0] = velocity;
    }
    let view = joint.view(data);
    if view.qpos.len() != 1 || view.qvel.len() != 1 {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "joint {joint_name} is not scalar"
        )));
    }
    let control = joint_control(world, entity, revolute, view.qpos[0], view.qvel[0])?;
    let actuator_id = data
        .model()
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, actuator_name)
        .ok_or_else(|| {
            MuJoCoError::UnsupportedFixture(format!("missing actuator {actuator_name}"))
        })?;
    data.ctrl_mut()[actuator_id] = control;
    Ok(())
}

fn joint_control(
    world: &World,
    entity: rne_ecs::Entity,
    revolute: bool,
    position: f64,
    velocity: f64,
) -> Result<f64, MuJoCoError> {
    if let Some(command) = world.get::<JointActuation>(entity).copied() {
        if !command.has_valid_values()
            || (revolute && !command.supports_revolute())
            || (!revolute && !command.supports_prismatic())
        {
            return Err(MuJoCoError::InvalidActuation {
                entity_index: entity.index(),
                reason: "mode, value, gain, or limit",
            });
        }
        let (effort, limit, passive_damping) = match command {
            JointActuation::Disabled => (0.0, 0.0, 0.0),
            JointActuation::RevolutePosition {
                target_position_rad,
                stiffness_nm_per_rad,
                damping_nm_s_per_rad,
                max_effort_nm,
            } => (
                stiffness_nm_per_rad * (target_position_rad - position)
                    - damping_nm_s_per_rad * velocity,
                max_effort_nm,
                damping_nm_s_per_rad,
            ),
            JointActuation::RevoluteVelocity {
                target_velocity_rad_s,
                gain_nm_s_per_rad,
                max_effort_nm,
            } => (
                gain_nm_s_per_rad * (target_velocity_rad_s - velocity),
                max_effort_nm,
                gain_nm_s_per_rad,
            ),
            JointActuation::RevoluteEffort {
                effort_nm,
                max_effort_nm,
            } => (effort_nm, max_effort_nm, 0.0),
            JointActuation::PrismaticPosition {
                target_position_m,
                stiffness_n_per_m,
                damping_n_s_per_m,
                max_force_n,
            } => (
                stiffness_n_per_m * (target_position_m - position) - damping_n_s_per_m * velocity,
                max_force_n,
                damping_n_s_per_m,
            ),
            JointActuation::PrismaticVelocity {
                target_velocity_m_s,
                gain_n_s_per_m,
                max_force_n,
            } => (
                gain_n_s_per_m * (target_velocity_m_s - velocity),
                max_force_n,
                gain_n_s_per_m,
            ),
            JointActuation::PrismaticEffort {
                force_n,
                max_force_n,
            } => (force_n, max_force_n, 0.0),
        };
        // MuJoCo integrates joint damping implicitly. Add the same damping term
        // back to the motor command after clamping so motor + passive damping
        // still realizes the exact bounded backend-neutral effort law.
        return Ok(effort.clamp(-limit, limit) + passive_damping * velocity);
    }
    let Some(motor) = world.get::<JointMotor>(entity) else {
        return Ok(0.0);
    };
    if !motor.velocity_rad_s.is_finite()
        || !motor.gain.is_finite()
        || !motor.stiffness.is_finite()
        || !motor.target_position.is_finite()
        || !motor.max_force.is_finite()
        || motor.gain < 0.0
        || motor.stiffness < 0.0
        || motor.max_force < 0.0
    {
        return Err(MuJoCoError::InvalidActuation {
            entity_index: entity.index(),
            reason: "legacy JointMotor value, gain, or limit",
        });
    }
    // The compiled joint applies `-damping * velocity` as MuJoCo-native passive
    // damping, which `implicitfast` treats implicitly. Keeping only the target
    // velocity feed-forward here is algebraically the same legacy PD law while
    // avoiding an explicit high-gain damping force on lightweight robot links.
    // `JointActuation` above remains exact and is used by conformance fixtures.
    let (stiffness, damping) = legacy_motor_gains(*motor, revolute);
    let effort = stiffness * (motor.target_position - position) + damping * motor.velocity_rad_s;
    Ok(if motor.max_force > 0.0 {
        effort.clamp(-motor.max_force, motor.max_force)
    } else {
        effort
    })
}

impl PhysicsBackend for MuJoCoBackend {
    type BodyHandle = MuJoCoBodyHandle;
    type ColliderHandle = MuJoCoColliderHandle;

    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
        if !desc.gravity_m_s2.x.is_finite()
            || !desc.gravity_m_s2.y.is_finite()
            || !desc.gravity_m_s2.z.is_finite()
        {
            return Err(Self::map_error(MuJoCoError::NonFiniteState("gravity")));
        }
        let (data, timestep_s, caller_mjcf) = match &self.model_source {
            ModelSource::EcsCompiler { timestep_s } => (None, *timestep_s, false),
            ModelSource::CallerMjcf(mjcf) => {
                let model = MjModel::from_xml_string(mjcf)
                    .map_err(|error| Self::map_error(MuJoCoError::ModelLoad(error.to_string())))?;
                require_free_fall_model(&model).map_err(Self::map_error)?;
                let mut data = MjData::try_new(Box::new(model))
                    .map_err(|_| Self::map_error(MuJoCoError::DataAllocation))?;
                data.model_opt_mut().gravity = [
                    desc.gravity_m_s2.x,
                    desc.gravity_m_s2.y,
                    desc.gravity_m_s2.z,
                ];
                let timestep_s = data.model_opt().timestep;
                (Some(data), timestep_s, true)
            }
        };
        let id = PhysicsWorldId(self.next_world_id);
        self.next_world_id = self.next_world_id.saturating_add(1);
        self.worlds.insert(
            id,
            MuJoCoWorld {
                data: Mutex::new(data),
                desc,
                bindings: Vec::new(),
                topology: Vec::new(),
                joint_dynamics: Vec::new(),
                caller_mjcf,
                timestep_s,
                geom_entities: Vec::new(),
                sensor_geoms: Vec::new(),
                contacts: Vec::new(),
            },
        );
        Ok(id)
    }

    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let caller_mjcf = self.world(physics_world)?.caller_mjcf;
        let compiled = if caller_mjcf {
            validate_caller_fixture_world(world)
        } else {
            let world_state = self.world(physics_world)?;
            compile_rigid_body_model(world, world_state.desc, world_state.timestep_s)
                .map_err(map_compile_error)
        }
        .map_err(Self::map_error)?;

        let world_state = self.world_mut(physics_world)?;
        if !world_state.topology.is_empty() && world_state.topology != compiled.topology {
            let detail = world_state
                .topology
                .iter()
                .zip(&compiled.topology)
                .enumerate()
                .find(|(_, (expected, actual))| expected != actual)
                .map(|(index, (expected, actual))| {
                    format!("entry {index}: expected {expected:?}, got {actual:?}")
                })
                .unwrap_or_else(|| {
                    format!(
                        "body count changed from {} to {}",
                        world_state.topology.len(),
                        compiled.topology.len()
                    )
                });
            return Err(Self::map_error(MuJoCoError::TopologyChanged { detail }));
        }
        if world_state.lock_data().is_none()
            || world_state.joint_dynamics != compiled.joint_dynamics
        {
            let model = MjModel::from_xml_string(&compiled.mjcf)
                .map_err(|error| Self::map_error(MuJoCoError::ModelLoad(error.to_string())))?;
            require_compiled_model(&model, &compiled.bindings).map_err(Self::map_error)?;
            let data = MjData::try_new(Box::new(model))
                .map_err(|_| Self::map_error(MuJoCoError::DataAllocation))?;
            *world_state.lock_data() = Some(data);
        }
        world_state.bindings = compiled.bindings;
        world_state.topology = compiled.topology;
        world_state.joint_dynamics = compiled.joint_dynamics;
        let (geom_entities, sensor_geoms) = {
            let data_guard = world_state.lock_data();
            let data = data_guard
                .as_ref()
                .ok_or(PhysicsError::InitializationFailed)?;
            geometry_bindings(data, &world_state.bindings, world).map_err(Self::map_error)?
        };
        world_state.geom_entities = geom_entities;
        world_state.sensor_geoms = sensor_geoms;
        {
            let mut data_guard = world_state.lock_data();
            let data = data_guard
                .as_mut()
                .ok_or(PhysicsError::InitializationFailed)?;
            sync_from_ecs_state(data, &world_state.bindings, world).map_err(Self::map_error)?;
        }
        Ok(())
    }

    fn step(&mut self, physics_world: PhysicsWorldId, dt: SimDuration) -> Result<(), PhysicsError> {
        let world_state = self.world_mut(physics_world)?;
        let actual_s = dt.as_seconds().value();
        if !actual_s.is_finite() || actual_s <= 0.0 {
            return Err(Self::map_error(MuJoCoError::InvalidInput(
                "step duration must be finite and positive".to_owned(),
            )));
        }
        if (actual_s - world_state.timestep_s).abs() > 1.0e-12 {
            return Err(Self::map_error(MuJoCoError::TimestepMismatch {
                expected_s: world_state.timestep_s,
                actual_s,
            }));
        }
        let contacts = {
            let mut data_guard = world_state.lock_data();
            let data = data_guard
                .as_mut()
                .ok_or(PhysicsError::InitializationFailed)?;
            data.step();
            if !data
                .qpos()
                .iter()
                .chain(data.qvel().iter())
                .all(|value| value.is_finite())
            {
                return Err(Self::map_error(MuJoCoError::NonFiniteState("qpos/qvel")));
            }
            collect_contact_events(
                data,
                &world_state.geom_entities,
                &world_state.sensor_geoms,
                world_state.timestep_s,
            )
            .map_err(Self::map_error)?
        };
        world_state.contacts = contacts;
        Ok(())
    }

    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let world_state = self.world(physics_world)?;
        let data_guard = world_state.lock_data();
        let data = data_guard
            .as_ref()
            .ok_or(PhysicsError::InitializationFailed)?;
        for binding in &world_state.bindings {
            match &binding.joint {
                JointBinding::Free { joint_name } => {
                    let joint = data
                        .joint(joint_name)
                        .ok_or(PhysicsError::InitializationFailed)?;
                    let joint_view = joint.view(data);
                    if joint_view.qpos.len() != EXPECTED_FREE_JOINT_QPOS_LEN
                        || joint_view.qvel.len() != EXPECTED_FREE_JOINT_QVEL_LEN
                        || !joint_view.qpos.iter().all(|value| value.is_finite())
                        || !joint_view.qvel.iter().all(|value| value.is_finite())
                    {
                        return Err(Self::map_error(MuJoCoError::NonFiniteState("body state")));
                    }
                    let rotation = Quat::from_xyzw(
                        joint_view.qpos[4],
                        joint_view.qpos[5],
                        joint_view.qpos[6],
                        joint_view.qpos[3],
                    );
                    if let Some(mut transform) = world.get_mut::<Transform3>(binding.entity) {
                        transform.translation = Vec3::from_slice(&joint_view.qpos[..3]);
                        transform.rotation = rotation;
                    }
                    if let Some(mut rigid_body) = world.get_mut::<RigidBody>(binding.entity) {
                        rigid_body.linear_velocity_m_s = Vec3::from_slice(&joint_view.qvel[..3]);
                        rigid_body.angular_velocity_rad_s =
                            Vec3::from_slice(&joint_view.qvel[3..6]);
                    }
                }
                JointBinding::Revolute { joint_name, .. }
                | JointBinding::Prismatic { joint_name, .. } => {
                    let joint = data
                        .joint(joint_name)
                        .ok_or(PhysicsError::InitializationFailed)?;
                    let joint_view = joint.view(data);
                    if joint_view.qpos.len() != 1
                        || joint_view.qvel.len() != 1
                        || !joint_view.qpos[0].is_finite()
                        || !joint_view.qvel[0].is_finite()
                    {
                        return Err(Self::map_error(MuJoCoError::NonFiniteState("joint state")));
                    }
                    let joint_state = match binding.joint {
                        JointBinding::Revolute { .. } => JointState::Revolute {
                            position_rad: joint_view.qpos[0],
                            velocity_rad_s: joint_view.qvel[0],
                        },
                        JointBinding::Prismatic { .. } => JointState::Prismatic {
                            position_m: joint_view.qpos[0],
                            velocity_m_s: joint_view.qvel[0],
                        },
                        JointBinding::Free { .. } | JointBinding::Fixed => unreachable!(),
                    };
                    let body = data
                        .body(&binding.body_name)
                        .ok_or(PhysicsError::InitializationFailed)?;
                    let body_view = body.view(data);
                    let rotation = Quat::from_xyzw(
                        body_view.xquat[1],
                        body_view.xquat[2],
                        body_view.xquat[3],
                        body_view.xquat[0],
                    );
                    if !rotation.is_finite()
                        || !body_view.xpos.iter().all(|value| value.is_finite())
                    {
                        return Err(Self::map_error(MuJoCoError::NonFiniteState(
                            "articulated body pose",
                        )));
                    }
                    let world_transform = Transform3::from_translation_rotation(
                        Vec3::from_slice(&body_view.xpos),
                        rotation,
                    );
                    let local_transform = world
                        .get::<Parent>(binding.entity)
                        .map(|parent| {
                            let parent_world = world_transform_of(world, parent.0);
                            let inverse_rotation = parent_world.rotation.conjugate();
                            Transform3::from_translation_rotation(
                                inverse_rotation
                                    * (world_transform.translation - parent_world.translation),
                                (inverse_rotation * world_transform.rotation).normalize(),
                            )
                        })
                        .unwrap_or(world_transform);
                    if let Some(mut transform) = world.get_mut::<Transform3>(binding.entity) {
                        *transform = local_transform;
                    }
                    world.entity_mut(binding.entity).insert(joint_state);
                }
                JointBinding::Fixed => {}
            }
        }
        Ok(())
    }

    fn raycast(
        &self,
        physics_world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let world_state = self.world(physics_world)?;
        let direction = query.direction;
        if direction.length_squared() <= f64::EPSILON {
            return Ok(Vec::new());
        }
        let direction = direction.normalize();
        let origin = query.origin_m;
        let pnt = [origin.x, origin.y, origin.z];
        let vec = [direction.x, direction.y, direction.z];
        let geom_entities = world_state.geom_entities.clone();
        let max_hits = geom_entities.len().max(1);

        let mut data_guard = world_state.lock_data();
        let Some(data) = data_guard.as_mut() else {
            return Ok(Vec::new());
        };
        let geom_bodyid = data.model().geom_bodyid().to_vec();

        // `mj_ray` returns only the nearest hit. Walk farther hits by excluding
        // each previously hit body so the batch contract matches Rapier.
        let mut hits = Vec::new();
        let mut excluded_body: Option<usize> = None;
        for _ in 0..max_hits {
            let mut normal = [0.0_f64; 3];
            let (geom_id, distance_m) =
                data.ray(&pnt, &vec, None, true, excluded_body, Some(&mut normal));
            if distance_m < 0.0 || distance_m > query.max_distance_m {
                break;
            }
            let Some(geom_id) = geom_id else {
                break;
            };
            excluded_body = geom_bodyid
                .get(geom_id)
                .copied()
                .and_then(|id| usize::try_from(id).ok());
            let Some(entity) = geom_entities.get(geom_id).copied().flatten() else {
                continue;
            };
            if !normal.iter().all(|value| value.is_finite()) || !distance_m.is_finite() {
                return Err(Self::map_error(MuJoCoError::NonFiniteState("raycast hit")));
            }
            hits.push(RaycastHit {
                entity,
                point_m: origin + direction * distance_m,
                normal: Vec3::new(normal[0], normal[1], normal[2]),
                distance_m,
            });
        }

        hits.sort_by(|left, right| {
            left.distance_m
                .total_cmp(&right.distance_m)
                .then_with(|| left.entity.index().cmp(&right.entity.index()))
        });
        Ok(hits)
    }

    fn contacts(&self, physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Ok(&self.world(physics_world)?.contacts)
    }

    fn capabilities(&self) -> &[PhysicsCapability] {
        CAPABILITIES
    }
}
