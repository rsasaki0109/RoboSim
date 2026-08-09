//! Live SUMO co-simulation bridge.
//!
//! [`CoSimulation`] drives a running SUMO process through [`crate::TraciClient`]
//! and mirrors every SUMO vehicle into an RNE ECS [`World`] as a
//! [`rne_traffic::TrafficActor`] with a [`rne_traffic::TrafficPose`] in the RNE
//! Y-up frame. SUMO owns the motion and routing; RNE owns the mirror, so RNE
//! sensors, logging, and rendering can observe live SUMO traffic.

use crate::{TraciClient, TraciError};
use rne_ecs::{Entity, EntityUuid, Name, World};
use rne_traffic::{TrafficActor, TrafficPose};
use std::collections::{BTreeMap, BTreeSet};

/// Stable namespace prefix so SUMO vehicle UUIDs never collide with random v4
/// entity UUIDs.
const SUMO_NAMESPACE: u128 = 0x6e6f_0000_0000_0000_0000_0000_0000_0000;

/// Mirrors SUMO vehicles as RNE traffic actors.
///
/// Actor entities carry [`Name`] equal to the SUMO vehicle id and a stable
/// [`EntityUuid`] derived from it, so replay and external iteration are
/// deterministic. Vehicles that leave the SUMO simulation are despawned.
pub struct CoSimulation {
    client: TraciClient,
    actors: BTreeMap<String, Entity>,
}

impl CoSimulation {
    /// Connects to a running SUMO process.
    pub fn connect(host: &str, port: u16) -> Result<Self, TraciError> {
        Ok(Self::from_client(TraciClient::connect(host, port)?))
    }

    /// Wraps an already-connected TraCI client.
    ///
    /// Useful when the caller performs connection retries or needs the raw
    /// client first.
    pub fn from_client(client: TraciClient) -> Self {
        Self {
            client,
            actors: BTreeMap::new(),
        }
    }

    /// Advances SUMO by one step and synchronizes the mirror actors.
    pub fn step(&mut self, world: &mut World) -> Result<(), TraciError> {
        self.client.simulation_step()?;
        let ids = self.client.vehicle_ids()?;
        let mut seen = BTreeSet::new();
        for id in &ids {
            let position = self.client.vehicle_position_rne(id)?;
            match self.actors.get(id) {
                Some(entity) => {
                    if let Some(mut pose) = world.get_mut::<TrafficPose>(*entity) {
                        pose.position_m = position;
                    }
                }
                None => {
                    let entity = world
                        .spawn((
                            Name::new(id),
                            TrafficActor::motor_vehicle(),
                            EntityUuid(stable_uuid(id)),
                            TrafficPose {
                                position_m: position,
                                yaw_rad: 0.0,
                            },
                        ))
                        .id();
                    self.actors.insert(id.clone(), entity);
                }
            }
            seen.insert(id.clone());
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
            }
        }
        Ok(())
    }

    /// The ECS entities mirroring the current SUMO vehicles, keyed by SUMO id.
    pub fn actors(&self) -> &BTreeMap<String, Entity> {
        &self.actors
    }

    /// Tells SUMO to close the connection and shut down.
    pub fn close(&mut self) -> Result<(), TraciError> {
        self.client.close()
    }
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
