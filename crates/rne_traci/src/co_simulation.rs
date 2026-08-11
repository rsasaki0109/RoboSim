//! Live SUMO co-simulation bridge.
//!
//! [`CoSimulation`] drives a running SUMO process through [`crate::TraciClient`]
//! and mirrors every SUMO vehicle into an RNE ECS [`World`] as a
//! [`rne_traffic::TrafficActor`] with a [`rne_traffic::TrafficPose`] in the RNE
//! Y-up frame. SUMO owns the motion and routing; RNE owns the mirror, so RNE
//! sensors, logging, and rendering can observe live SUMO traffic. Mirrored
//! actors are tagged with [`rne_traffic::TrafficPoseSource::External`] so a
//! concurrent RNE traffic runtime does not integrate the same pose twice.

use crate::{TraciClient, TraciError};
use rne_ecs::{Entity, EntityUuid, Name, World};
use rne_traffic::{TrafficActor, TrafficPose, TrafficPoseSource};
use std::collections::{BTreeMap, BTreeSet};

/// Stable namespace prefix so SUMO vehicle UUIDs never collide with random v4
/// entity UUIDs.
const SUMO_NAMESPACE: u128 = 0x6e6f_0000_0000_0000_0000_0000_0000_0000;

/// Network endpoint used to reconnect a live TraCI session.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TraciEndpoint {
    host: String,
    port: u16,
}

impl TraciEndpoint {
    /// Creates a validated endpoint.
    pub fn new(host: impl Into<String>, port: u16) -> Result<Self, TraciError> {
        let host = host.into();
        if host.trim().is_empty() || port == 0 {
            return Err(TraciError::InvalidArgument(
                "TraCI endpoint requires a non-empty host and non-zero port".to_string(),
            ));
        }
        Ok(Self { host, port })
    }

    /// Endpoint host name or address.
    pub fn host(&self) -> &str {
        &self.host
    }

    /// Endpoint TCP port.
    pub const fn port(&self) -> u16 {
        self.port
    }
}

/// Explicit lifecycle state of a co-simulation connection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoSimulationSessionState {
    /// Commands and simulation steps may be sent.
    Connected,
    /// The last complete mirror is retained, but a replacement client is required.
    Disconnected,
    /// The session has been closed permanently.
    Closed,
}

impl CoSimulationSessionState {
    fn as_str(self) -> &'static str {
        match self {
            Self::Connected => "connected",
            Self::Disconnected => "disconnected",
            Self::Closed => "closed",
        }
    }
}

/// Bounded reconnect settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReconnectPolicy {
    max_attempts: u32,
}

impl ReconnectPolicy {
    /// Creates a policy with a non-zero attempt bound.
    pub fn new(max_attempts: u32) -> Result<Self, TraciError> {
        if max_attempts == 0 {
            return Err(TraciError::InvalidArgument(
                "TraCI reconnect max_attempts must be non-zero".to_string(),
            ));
        }
        Ok(Self { max_attempts })
    }

    /// Maximum replacement connections attempted by one recovery call.
    pub const fn max_attempts(self) -> u32 {
        self.max_attempts
    }
}

impl Default for ReconnectPolicy {
    fn default() -> Self {
        Self { max_attempts: 3 }
    }
}

/// Monotonic diagnostics for one co-simulation session.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CoSimulationSessionMetrics {
    /// Current lifecycle state.
    pub state: CoSimulationSessionState,
    /// Connection generation, starting at one and incremented after recovery.
    pub generation: u64,
    /// Steps whose complete vehicle snapshot was committed to ECS.
    pub successful_steps: u64,
    /// Step calls that returned an error before committing a snapshot.
    pub failed_steps: u64,
    /// Replacement connections attempted across all recoveries.
    pub reconnect_attempts: u64,
    /// Recoveries whose snapshot was committed successfully.
    pub successful_recoveries: u64,
}

/// Result of one successful snapshot-only recovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CoSimulationRecovery {
    /// New connection generation.
    pub generation: u64,
    /// Replacement connections attempted by this recovery call.
    pub attempts: u32,
    /// Mirror actors created from the replacement snapshot.
    pub created_actor_count: usize,
    /// Existing mirror actors updated from the replacement snapshot.
    pub updated_actor_count: usize,
    /// Stale mirror actors removed by the replacement snapshot.
    pub removed_actor_count: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct MirrorDelta {
    created_actor_count: usize,
    updated_actor_count: usize,
    removed_actor_count: usize,
}

/// Mirrors SUMO vehicles as RNE traffic actors.
///
/// Actor entities carry [`Name`] equal to the SUMO vehicle id and a stable
/// [`EntityUuid`] derived from it, so replay and external iteration are
/// deterministic. Vehicles that leave the SUMO simulation are despawned.
pub struct CoSimulation {
    client: Option<TraciClient>,
    endpoint: Option<TraciEndpoint>,
    actors: BTreeMap<String, Entity>,
    state: CoSimulationSessionState,
    generation: u64,
    successful_steps: u64,
    failed_steps: u64,
    reconnect_attempts: u64,
    successful_recoveries: u64,
}

impl CoSimulation {
    /// Connects to a running SUMO process.
    pub fn connect(host: &str, port: u16) -> Result<Self, TraciError> {
        let endpoint = TraciEndpoint::new(host, port)?;
        let client = TraciClient::connect(endpoint.host(), endpoint.port())?;
        Ok(Self::from_parts(client, Some(endpoint)))
    }

    /// Wraps an already-connected TraCI client.
    ///
    /// Useful when the caller performs connection retries or needs the raw
    /// client first.
    pub fn from_client(client: TraciClient) -> Self {
        Self::from_parts(client, None)
    }

    fn from_parts(client: TraciClient, endpoint: Option<TraciEndpoint>) -> Self {
        Self {
            client: Some(client),
            endpoint,
            actors: BTreeMap::new(),
            state: CoSimulationSessionState::Connected,
            generation: 1,
            successful_steps: 0,
            failed_steps: 0,
            reconnect_attempts: 0,
            successful_recoveries: 0,
        }
    }

    /// Advances SUMO by one step and synchronizes the mirror actors.
    pub fn step(&mut self, world: &mut World) -> Result<(), TraciError> {
        self.require_state("step", CoSimulationSessionState::Connected)?;
        let result = {
            let client = self
                .client
                .as_mut()
                .expect("connected session has a client");
            client
                .simulation_step()
                .and_then(|()| read_vehicle_snapshot(client))
        };
        let positions = match result {
            Ok(positions) => positions,
            Err(error) => {
                self.failed_steps += 1;
                self.handle_session_failure(&error);
                return Err(error);
            }
        };
        self.apply_snapshot(world, positions);
        self.successful_steps += 1;
        Ok(())
    }

    /// Reads and commits the current SUMO vehicle snapshot without advancing SUMO.
    pub fn resynchronize(&mut self, world: &mut World) -> Result<(), TraciError> {
        self.require_state("resynchronize", CoSimulationSessionState::Connected)?;
        let result = {
            let client = self
                .client
                .as_mut()
                .expect("connected session has a client");
            read_vehicle_snapshot(client)
        };
        let positions = match result {
            Ok(positions) => positions,
            Err(error) => {
                self.handle_session_failure(&error);
                return Err(error);
            }
        };
        self.apply_snapshot(world, positions);
        Ok(())
    }

    /// Recovers a disconnected endpoint-backed session with bounded attempts.
    ///
    /// Recovery reads a complete vehicle snapshot before replacing the failed
    /// client and never sends `simulationStep`.
    pub fn recover(
        &mut self,
        world: &mut World,
        policy: ReconnectPolicy,
    ) -> Result<CoSimulationRecovery, TraciError> {
        self.require_state("recover", CoSimulationSessionState::Disconnected)?;
        let endpoint = self
            .endpoint
            .clone()
            .ok_or(TraciError::RecoveryUnavailable)?;
        let mut last_error = "no reconnect attempt was made".to_string();
        for attempt in 1..=policy.max_attempts() {
            self.reconnect_attempts += 1;
            let result =
                TraciClient::connect(endpoint.host(), endpoint.port()).and_then(|mut client| {
                    read_vehicle_snapshot(&mut client).map(|snapshot| (client, snapshot))
                });
            match result {
                Ok((client, snapshot)) => {
                    return Ok(self.complete_recovery(world, client, snapshot, attempt));
                }
                Err(error) => last_error = error.to_string(),
            }
        }
        Err(TraciError::RecoveryFailed {
            attempts: policy.max_attempts(),
            last_error,
        })
    }

    /// Recovers a disconnected session from a caller-provided replacement client.
    ///
    /// This is the recovery path for sessions created with [`Self::from_client`].
    pub fn recover_from_client(
        &mut self,
        world: &mut World,
        mut client: TraciClient,
    ) -> Result<CoSimulationRecovery, TraciError> {
        self.require_state(
            "recover_from_client",
            CoSimulationSessionState::Disconnected,
        )?;
        self.reconnect_attempts += 1;
        let snapshot = read_vehicle_snapshot(&mut client)?;
        Ok(self.complete_recovery(world, client, snapshot, 1))
    }

    fn complete_recovery(
        &mut self,
        world: &mut World,
        client: TraciClient,
        snapshot: BTreeMap<String, [f64; 3]>,
        attempts: u32,
    ) -> CoSimulationRecovery {
        let delta = self.apply_snapshot(world, snapshot);
        self.client = Some(client);
        self.state = CoSimulationSessionState::Connected;
        self.generation += 1;
        self.successful_recoveries += 1;
        CoSimulationRecovery {
            generation: self.generation,
            attempts,
            created_actor_count: delta.created_actor_count,
            updated_actor_count: delta.updated_actor_count,
            removed_actor_count: delta.removed_actor_count,
        }
    }

    fn apply_snapshot(
        &mut self,
        world: &mut World,
        positions: BTreeMap<String, [f64; 3]>,
    ) -> MirrorDelta {
        let seen = positions.keys().cloned().collect::<BTreeSet<_>>();
        let mut delta = MirrorDelta::default();

        for (id, position) in positions {
            match self.actors.get(&id).copied() {
                Some(entity) if world.get::<TrafficPose>(entity).is_some() => {
                    if let Some(mut pose) = world.get_mut::<TrafficPose>(entity) {
                        pose.position_m = position;
                    }
                    delta.updated_actor_count += 1;
                }
                _ => {
                    let entity = world
                        .spawn((
                            Name::new(&id),
                            TrafficActor::motor_vehicle(),
                            TrafficPoseSource::External,
                            EntityUuid(stable_uuid(&id)),
                            TrafficPose {
                                position_m: position,
                                yaw_rad: 0.0,
                            },
                        ))
                        .id();
                    self.actors.insert(id, entity);
                    delta.created_actor_count += 1;
                }
            }
        }
        let departed = self
            .actors
            .keys()
            .filter(|id| !seen.contains(*id))
            .cloned()
            .collect::<Vec<_>>();
        for id in departed {
            if let Some(entity) = self.actors.remove(&id) {
                let _ = world.despawn(entity);
                delta.removed_actor_count += 1;
            }
        }
        delta
    }

    /// The ECS entities mirroring the current SUMO vehicles, keyed by SUMO id.
    pub fn actors(&self) -> &BTreeMap<String, Entity> {
        &self.actors
    }

    /// Current lifecycle state.
    pub const fn state(&self) -> CoSimulationSessionState {
        self.state
    }

    /// Current endpoint, when automatic recovery is available.
    pub fn endpoint(&self) -> Option<&TraciEndpoint> {
        self.endpoint.as_ref()
    }

    /// Monotonic session diagnostics.
    pub const fn metrics(&self) -> CoSimulationSessionMetrics {
        CoSimulationSessionMetrics {
            state: self.state,
            generation: self.generation,
            successful_steps: self.successful_steps,
            failed_steps: self.failed_steps,
            reconnect_attempts: self.reconnect_attempts,
            successful_recoveries: self.successful_recoveries,
        }
    }

    /// Explicitly returns a vehicle speed command to SUMO.
    ///
    /// This is an opt-in control path: calling [`Self::step`] never derives or
    /// sends commands from the mirrored RNE pose. SUMO still owns vehicle
    /// integration and routing, while the RNE traffic runtime continues to
    /// treat the mirrored actor as [`TrafficPoseSource::External`].
    pub fn set_vehicle_speed_m_s(
        &mut self,
        vehicle_id: &str,
        speed_m_s: f64,
    ) -> Result<(), TraciError> {
        self.require_state("set_vehicle_speed_m_s", CoSimulationSessionState::Connected)?;
        let result = self
            .client
            .as_mut()
            .expect("connected session has a client")
            .set_vehicle_speed_m_s(vehicle_id, speed_m_s);
        if let Err(error) = &result {
            self.handle_session_failure(error);
        }
        result
    }

    /// Tells SUMO to close the connection and shut down.
    pub fn close(&mut self) -> Result<(), TraciError> {
        if self.state == CoSimulationSessionState::Closed {
            return Ok(());
        }
        let result = if let Some(client) = self.client.as_mut() {
            client.close()
        } else {
            Ok(())
        };
        self.client = None;
        self.state = CoSimulationSessionState::Closed;
        result
    }

    fn require_state(
        &self,
        operation: &'static str,
        required: CoSimulationSessionState,
    ) -> Result<(), TraciError> {
        if self.state != required {
            return Err(TraciError::SessionState {
                operation,
                state: self.state.as_str(),
            });
        }
        Ok(())
    }

    fn handle_session_failure(&mut self, error: &TraciError) {
        if error.disconnects_session() {
            self.client = None;
            self.state = CoSimulationSessionState::Disconnected;
        }
    }
}

fn read_vehicle_snapshot(
    client: &mut TraciClient,
) -> Result<BTreeMap<String, [f64; 3]>, TraciError> {
    let mut ids = client.vehicle_ids()?;
    ids.sort();
    ids.into_iter()
        .map(|id| {
            client
                .vehicle_position_rne(&id)
                .map(|position| (id, position))
        })
        .collect()
}

/// Derives a stable entity UUID from a SUMO vehicle id.
fn stable_uuid(vehicle_id: &str) -> uuid::Uuid {
    const OFFSET: u128 = 144_066_263_297_769_815_596_495_629_667_062_367_629;
    const PRIME: u128 = 309_485_009_821_345_068_724_781_371;
    let hash = vehicle_id.as_bytes().iter().fold(OFFSET, |hash, byte| {
        (hash ^ u128::from(*byte)).wrapping_mul(PRIME)
    });
    uuid::Uuid::from_u128(SUMO_NAMESPACE | (hash & 0x0000_ffff_ffff_ffff_ffff_ffff_ffff_ffff))
}
