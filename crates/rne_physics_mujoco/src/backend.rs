//! Feature-gated MuJoCo backend implementation.

use crate::compiler::{
    compile_rigid_body_model, BodyBinding, BodyTopology, CompileError, CompiledRigidBodyModel,
};
use crate::EXPECTED_MUJOCO_VERSION_PREFIX;
use mujoco_rs::prelude::{MjData, MjModel, MjtObj};
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    ColliderShape, ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc,
    PhysicsWorldId, RaycastHit, RaycastQuery, RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use std::collections::HashMap;
use thiserror::Error;

const FREE_FALL_BODY_NAME: &str = "rne_free_fall_body";
const FREE_FALL_JOINT_NAME: &str = "rne_free_fall_joint";
const EXPECTED_FREE_JOINT_QPOS_LEN: usize = 7;
const EXPECTED_FREE_JOINT_QVEL_LEN: usize = 6;

const CAPABILITIES: &[PhysicsCapability] = &[PhysicsCapability::RigidBody];

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
    /// The ECS world requires an undeclared backend capability.
    #[error("MuJoCo backend lacks required capability {capability:?}")]
    MissingCapability {
        /// Capability required by the rejected ECS world.
        capability: PhysicsCapability,
    },
    /// The fixed topology changed after the native model was compiled.
    #[error("MuJoCo world topology changed after step 0")]
    TopologyChanged,
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
/// Contact reporting, raycasts, and articulation are separate capabilities.
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
    data: Option<MjData<Box<MjModel>>>,
    desc: PhysicsWorldDesc,
    bindings: Vec<BodyBinding>,
    topology: Vec<BodyTopology>,
    caller_mjcf: bool,
    timestep_s: f64,
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
    /// It reports articulation and sensor requirements explicitly and keeps
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
            MuJoCoError::MissingCapability { capability } => PhysicsError::MissingCapabilities {
                missing: vec![capability],
            },
            _ => PhysicsError::InitializationFailed,
        }
    }
}

fn map_compile_error(error: CompileError) -> MuJoCoError {
    match error {
        CompileError::MissingCapability(capability) => {
            MuJoCoError::MissingCapability { capability }
        }
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
    compiled.bindings[0].joint_name = Some(FREE_FALL_JOINT_NAME.to_owned());
    Ok(compiled)
}

fn require_compiled_model(model: &MjModel, bindings: &[BodyBinding]) -> Result<(), MuJoCoError> {
    let dynamic_count = bindings
        .iter()
        .filter(|binding| binding.joint_name.is_some())
        .count();
    let expected_nq = dynamic_count * EXPECTED_FREE_JOINT_QPOS_LEN;
    let expected_nv = dynamic_count * EXPECTED_FREE_JOINT_QVEL_LEN;
    if model.nq() as usize != expected_nq || model.nv() as usize != expected_nv {
        return Err(MuJoCoError::UnsupportedFixture(format!(
            "compiled free-joint dimensions must be nq={expected_nq}, nv={expected_nv}"
        )));
    }
    for binding in bindings {
        if let Some(name) = binding.joint_name.as_deref() {
            if model.name_to_id(MjtObj::mjOBJ_JOINT, name).is_none() {
                return Err(MuJoCoError::UnsupportedFixture(format!(
                    "compiled model is missing joint {name}"
                )));
            }
        }
    }
    Ok(())
}

fn sync_from_ecs_state(
    data: &mut MjData<Box<MjModel>>,
    bindings: &[BodyBinding],
    world: &World,
) -> Result<(), MuJoCoError> {
    for binding in bindings {
        let Some(joint_name) = binding.joint_name.as_deref() else {
            continue;
        };
        let rigid_body = world.get::<RigidBody>(binding.entity).ok_or_else(|| {
            MuJoCoError::UnsupportedFixture("rigid body disappeared during sync".to_owned())
        })?;
        let transform = world.get::<Transform3>(binding.entity).ok_or_else(|| {
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
    }
    data.forward();
    Ok(())
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
                data,
                desc,
                bindings: Vec::new(),
                topology: Vec::new(),
                caller_mjcf,
                timestep_s,
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
            return Err(Self::map_error(MuJoCoError::TopologyChanged));
        }
        if world_state.data.is_none() {
            let model = MjModel::from_xml_string(&compiled.mjcf)
                .map_err(|error| Self::map_error(MuJoCoError::ModelLoad(error.to_string())))?;
            require_compiled_model(&model, &compiled.bindings).map_err(Self::map_error)?;
            let data = MjData::try_new(Box::new(model))
                .map_err(|_| Self::map_error(MuJoCoError::DataAllocation))?;
            world_state.data = Some(data);
        }
        world_state.bindings = compiled.bindings;
        world_state.topology = compiled.topology;
        sync_from_ecs_state(
            world_state
                .data
                .as_mut()
                .ok_or(PhysicsError::InitializationFailed)?,
            &world_state.bindings,
            world,
        )
        .map_err(Self::map_error)
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
        let data = world_state
            .data
            .as_mut()
            .ok_or(PhysicsError::InitializationFailed)?;
        data.step();
        if data
            .qpos()
            .iter()
            .chain(data.qvel().iter())
            .all(|value| value.is_finite())
        {
            Ok(())
        } else {
            Err(Self::map_error(MuJoCoError::NonFiniteState("qpos/qvel")))
        }
    }

    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let world_state = self.world(physics_world)?;
        let data = world_state
            .data
            .as_ref()
            .ok_or(PhysicsError::InitializationFailed)?;
        for binding in &world_state.bindings {
            let Some(joint_name) = binding.joint_name.as_deref() else {
                continue;
            };
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
                rigid_body.angular_velocity_rad_s = Vec3::from_slice(&joint_view.qvel[3..6]);
            }
        }
        Ok(())
    }

    fn raycast(
        &self,
        _physics_world: PhysicsWorldId,
        _query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        Err(PhysicsError::MissingCapabilities {
            missing: vec![PhysicsCapability::RaycastBatch],
        })
    }

    fn contacts(&self, _physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Err(PhysicsError::MissingCapabilities {
            missing: vec![PhysicsCapability::ContactForce],
        })
    }

    fn capabilities(&self) -> &[PhysicsCapability] {
        CAPABILITIES
    }
}
