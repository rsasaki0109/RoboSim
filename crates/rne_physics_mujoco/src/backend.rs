//! Feature-gated MuJoCo backend implementation.

use crate::EXPECTED_MUJOCO_VERSION_PREFIX;
use mujoco_rs::prelude::{MjData, MjModel, MjtObj};
use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError,
    PhysicsWorldDesc, PhysicsWorldId, RaycastHit, RaycastQuery, RigidBody, RigidBodyType,
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

/// MuJoCo-backed rigid-body adapter for the bounded free-fall fixture.
///
/// MuJoCo model and data types remain private implementation details.  The
/// adapter intentionally supports one dynamic sphere with one free joint;
/// contact reporting, raycasts, articulated ECS synchronization, and general
/// ECS-to-MJCF compilation are deferred to later capability work.
#[derive(Debug)]
pub struct MuJoCoBackend {
    mjcf: String,
    worlds: HashMap<PhysicsWorldId, MuJoCoWorld>,
    next_world_id: u32,
}

#[derive(Debug)]
struct MuJoCoWorld {
    data: MjData<Box<MjModel>>,
    entity: Option<Entity>,
    timestep_s: f64,
}

impl MuJoCoBackend {
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
            mjcf,
            worlds: HashMap::new(),
            next_world_id: 0,
        })
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

    fn map_error(_error: MuJoCoError) -> PhysicsError {
        PhysicsError::InitializationFailed
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
        let model = MjModel::from_xml_string(&self.mjcf)
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
        let id = PhysicsWorldId(self.next_world_id);
        self.next_world_id = self.next_world_id.saturating_add(1);
        self.worlds.insert(
            id,
            MuJoCoWorld {
                data,
                entity: None,
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
        let mut bodies = world
            .iter_entities()
            .filter_map(|entity_ref| {
                let entity = entity_ref.id();
                let rigid_body = world.get::<RigidBody>(entity)?;
                let collider = world.get::<Collider>(entity)?;
                let transform = world.get::<Transform3>(entity)?;
                Some((entity, *rigid_body, *collider, *transform))
            })
            .collect::<Vec<_>>();
        bodies.sort_unstable_by_key(|(entity, _, _, _)| entity.index());
        let Some((entity, rigid_body, collider, transform)) = bodies.first().copied() else {
            return Err(Self::map_error(MuJoCoError::UnsupportedFixture(
                "the ECS world must contain one rigid body and collider".to_owned(),
            )));
        };
        if bodies.len() != 1 {
            return Err(Self::map_error(MuJoCoError::UnsupportedFixture(
                "the spike accepts exactly one ECS body".to_owned(),
            )));
        }
        if rigid_body.body_type != RigidBodyType::Dynamic {
            return Err(Self::map_error(MuJoCoError::UnsupportedFixture(
                "the free-fall body must be dynamic".to_owned(),
            )));
        }
        if !rigid_body.mass_kg.is_finite() || rigid_body.mass_kg <= 0.0 {
            return Err(Self::map_error(MuJoCoError::InvalidInput(
                "body mass must be finite and positive".to_owned(),
            )));
        }
        if !matches!(collider.shape, ColliderShape::Sphere { .. }) {
            return Err(Self::map_error(MuJoCoError::UnsupportedFixture(
                "the spike accepts only a sphere collider".to_owned(),
            )));
        }
        finite_vec3(transform.translation, "position").map_err(Self::map_error)?;
        finite_quat(transform.rotation, "rotation").map_err(Self::map_error)?;
        finite_vec3(rigid_body.linear_velocity_m_s, "linear velocity").map_err(Self::map_error)?;
        finite_vec3(rigid_body.angular_velocity_rad_s, "angular velocity")
            .map_err(Self::map_error)?;

        let state = (entity, rigid_body, transform);
        let world_state = self.world_mut(physics_world)?;
        let joint = world_state
            .data
            .joint(FREE_FALL_JOINT_NAME)
            .ok_or(PhysicsError::InitializationFailed)?;
        let mut joint_view = joint.view_mut(&mut world_state.data);
        if joint_view.qpos.len() != EXPECTED_FREE_JOINT_QPOS_LEN
            || joint_view.qvel.len() != EXPECTED_FREE_JOINT_QVEL_LEN
        {
            return Err(PhysicsError::InitializationFailed);
        }
        joint_view.qpos[..3].copy_from_slice(&[
            state.2.translation.x,
            state.2.translation.y,
            state.2.translation.z,
        ]);
        joint_view.qpos[3..7].copy_from_slice(&[
            state.2.rotation.w,
            state.2.rotation.x,
            state.2.rotation.y,
            state.2.rotation.z,
        ]);
        joint_view.qvel[..3].copy_from_slice(&[
            state.1.linear_velocity_m_s.x,
            state.1.linear_velocity_m_s.y,
            state.1.linear_velocity_m_s.z,
        ]);
        joint_view.qvel[3..6].copy_from_slice(&[
            state.1.angular_velocity_rad_s.x,
            state.1.angular_velocity_rad_s.y,
            state.1.angular_velocity_rad_s.z,
        ]);
        world_state.entity = Some(state.0);
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
        world_state.data.step();
        if world_state
            .data
            .qpos()
            .iter()
            .chain(world_state.data.qvel().iter())
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
        let Some(entity) = world_state.entity else {
            return Err(PhysicsError::InitializationFailed);
        };
        let joint = world_state
            .data
            .joint(FREE_FALL_JOINT_NAME)
            .ok_or(PhysicsError::InitializationFailed)?;
        let joint_view = joint.view(&world_state.data);
        if joint_view.qpos.len() == EXPECTED_FREE_JOINT_QPOS_LEN
            && joint_view.qvel.len() == EXPECTED_FREE_JOINT_QVEL_LEN
            && joint_view.qpos.iter().all(|value| value.is_finite())
            && joint_view.qvel.iter().all(|value| value.is_finite())
        {
            let rotation = Quat::from_xyzw(
                joint_view.qpos[4],
                joint_view.qpos[5],
                joint_view.qpos[6],
                joint_view.qpos[3],
            );
            if let Some(mut transform) = world.get_mut::<Transform3>(entity) {
                transform.translation = Vec3::from_slice(&joint_view.qpos[..3]);
                transform.rotation = rotation;
            }
            if let Some(mut rigid_body) = world.get_mut::<RigidBody>(entity) {
                rigid_body.linear_velocity_m_s = Vec3::from_slice(&joint_view.qvel[..3]);
                rigid_body.angular_velocity_rad_s = Vec3::from_slice(&joint_view.qvel[3..6]);
            }
            Ok(())
        } else {
            Err(Self::map_error(MuJoCoError::NonFiniteState("body state")))
        }
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
