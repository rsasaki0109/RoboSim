//! Deterministic scenario execution over the traffic runtime.
//!
//! The executor turns a [`ScenarioDocument`] and a [`TrafficNetwork`] into a
//! deterministic fixed-step run: it derives one actor-compatible route from the
//! network, spawns the scenario entities as traffic actors on that route, and
//! applies each storyboard speed action at its scheduled simulation time while
//! stepping the kinematic traffic systems.

use crate::{ScenarioAction, ScenarioDocument, ScenarioEntityKind, ScenarioError};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{EntityUuid, Name, World};
use rne_math::Hertz;
use rne_traffic::{
    shortest_lane_route, TrafficActor, TrafficActorKind, TrafficId, TrafficNetwork, TrafficPose,
    TrafficRoute, TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime, TrafficSignalControls,
};
use std::collections::{BTreeMap, BTreeSet};
use uuid::Uuid;

/// Fixed-step execution settings for a scenario.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScenarioRunOptions {
    /// Number of fixed simulation steps.
    pub steps: u64,
    /// Fixed simulation rate in hertz.
    pub hz: f64,
}

impl Default for ScenarioRunOptions {
    fn default() -> Self {
        Self {
            steps: 600,
            hz: 60.0,
        }
    }
}

/// Deterministic outcome of one scenario execution.
#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioRunResult {
    /// Stable hash of the ordered actor state after the last step.
    pub stable_hash: u64,
    /// Red stop-line crossings across the run.
    pub signal_violations: usize,
    /// Overlapping bumper pairs observed across the run.
    pub collisions: usize,
    /// Actor positions after the last step, in actor spawn order.
    pub final_positions_m: Vec<[f64; 3]>,
    /// Length of the derived traffic route in metres.
    pub route_length_m: f64,
    /// Average speed of departed, unfinished actors after the last step.
    pub average_speed_m_s: f64,
}

/// Body length assumed per actor kind for traffic-following spacing.
pub fn actor_length_m(kind: ScenarioEntityKind) -> f64 {
    match kind {
        ScenarioEntityKind::MotorVehicle => 4.4,
        ScenarioEntityKind::Bicycle => 1.8,
        ScenarioEntityKind::Pedestrian => 0.5,
    }
}

fn actor_kind(kind: ScenarioEntityKind) -> TrafficActorKind {
    match kind {
        ScenarioEntityKind::MotorVehicle => TrafficActorKind::MotorVehicle,
        ScenarioEntityKind::Bicycle => TrafficActorKind::Bicycle,
        ScenarioEntityKind::Pedestrian => TrafficActorKind::Pedestrian,
    }
}

/// Executes a scenario document over a traffic network.
///
/// The runner derives a single route from the first and last network lanes that
/// admit the scenario's primary actor kind, spawns every entity on that route at
/// its initial position (or the route start), and applies the document's speed
/// actions at their scheduled simulation times. Unsignalized kinematics are
/// used; network signals and junction reservations are not yet applied.
pub fn execute_scenario(
    document: &ScenarioDocument,
    network: &TrafficNetwork,
    options: &ScenarioRunOptions,
) -> Result<ScenarioRunResult, ScenarioError> {
    document.validate()?;
    if !options.hz.is_finite() || options.hz <= 0.0 {
        return Err(ScenarioError::Invalid(
            "scenario run hz must be finite and positive".to_string(),
        ));
    }

    let primary_kind = document
        .entities
        .iter()
        .map(|entity| entity.kind)
        .next()
        .ok_or_else(|| ScenarioError::Invalid("scenario has no entities".to_string()))?;
    let route = derive_route(network, primary_kind)?;
    let parallel_route = parallel_route_for(&route, document)?;

    let mut world = World::new();
    let mut spawn_order = document.entities.clone();
    spawn_order.sort_by(|left, right| left.name.cmp(&right.name));
    for (index, entity) in spawn_order.iter().enumerate() {
        let distance_m = spawn_distance_m(&route, entity.initial_world_position_m);
        let pose = route.sample(distance_m);
        world.spawn((
            Name(entity.name.clone()),
            TrafficActor {
                kind: actor_kind(entity.kind),
            },
            EntityUuid(Uuid::from_u128(uuid_for_entity(index))),
            TrafficRouteFollower {
                route_id: route.id().clone(),
                distance_m,
                speed_m_s: 0.0,
                desired_speed_m_s: 0.0,
                length_m: actor_length_m(entity.kind),
            },
            TrafficPose {
                position_m: pose.position_m,
                yaw_rad: pose.yaw_rad,
            },
        ));
    }

    let mut routes = TrafficRouteCatalog::default();
    routes
        .insert(route.clone())
        .map_err(|error| ScenarioError::Invalid(format!("insert route: {error}")))?;
    if let Some(parallel_route) = &parallel_route {
        routes
            .insert(parallel_route.clone())
            .map_err(|error| ScenarioError::Invalid(format!("insert parallel route: {error}")))?;
    }
    let controls = TrafficSignalControls::default();
    let mut runtime = TrafficRuntime::default();
    let delta = SimDuration::from_hertz(Hertz::new(options.hz));
    let schedules = build_action_schedules(document);

    let mut violations = 0;
    let mut collisions = 0;
    let mut stable_hash = 0;
    let mut average_speed_m_s = 0.0;
    for step in 1..=options.steps {
        let sim_time = SimTime::from_ticks(step * delta.ticks());
        apply_due_actions(&mut world, &schedules, sim_time, parallel_route.as_ref());
        let report = rne_traffic::advance_controlled_kinematic_traffic(
            &mut world,
            &routes,
            &controls,
            &mut runtime,
            sim_time,
            delta,
            rne_traffic::KinematicTrafficConfig::default(),
        )
        .map_err(|error| ScenarioError::Invalid(format!("traffic step: {error}")))?;
        violations += report.signal_violation_count;
        collisions += report.collision_count;
        stable_hash = report.stable_state_hash;
        average_speed_m_s = report.flow.average_speed_m_s;
    }

    let mut positions = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let name = world.get::<Name>(entity)?;
            let pose = world.get::<TrafficPose>(entity)?;
            Some((name.0.clone(), pose.position_m))
        })
        .collect::<Vec<_>>();
    positions.sort_by(|left, right| left.0.cmp(&right.0));
    let final_positions_m = positions
        .into_iter()
        .map(|(_, position)| position)
        .collect();

    Ok(ScenarioRunResult {
        stable_hash,
        signal_violations: violations,
        collisions,
        final_positions_m,
        route_length_m: route.total_length_m(),
        average_speed_m_s,
    })
}

fn derive_route(
    network: &TrafficNetwork,
    kind: ScenarioEntityKind,
) -> Result<TrafficRoute, ScenarioError> {
    let traffic_asset = rne_traffic::TrafficAsset::new(network.clone());
    traffic_asset
        .validate()
        .map_err(|error| ScenarioError::Invalid(format!("traffic network: {error}")))?;

    let mut allowed_lanes = network
        .lanes
        .iter()
        .filter(|lane| lane.allowed_actors.contains(&actor_kind(kind)))
        .map(|lane| lane.id.clone())
        .collect::<Vec<_>>();
    allowed_lanes.sort_by(|left, right| left.as_str().cmp(right.as_str()));
    if allowed_lanes.is_empty() {
        return Err(ScenarioError::Invalid(format!(
            "network has no lanes that allow {:?}",
            kind
        )));
    }

    let incoming = network
        .connections
        .iter()
        .map(|connection| &connection.incoming_lane_id)
        .collect::<BTreeSet<_>>();
    let outgoing = network
        .connections
        .iter()
        .map(|connection| &connection.outgoing_lane_id)
        .collect::<BTreeSet<_>>();
    // A source lane has nothing flowing into it (no connection lists it as its
    // outgoing lane); a sink lane has nothing flowing out (no connection lists
    // it as its incoming lane). Try every source/sink pair in deterministic
    // order so directed networks resolve without guessing corridor orientation.
    let starts = allowed_lanes
        .iter()
        .filter(|lane_id| !outgoing.contains(lane_id))
        .collect::<Vec<_>>();
    let goals = allowed_lanes
        .iter()
        .filter(|lane_id| !incoming.contains(lane_id))
        .collect::<Vec<_>>();

    // Try every source/sink pair in deterministic order so directed networks
    // resolve without guessing the corridor orientation.
    for start_lane in starts.iter() {
        for goal_lane in goals.iter() {
            if let Ok(lane_route) =
                shortest_lane_route(network, start_lane, goal_lane, actor_kind(kind))
            {
                let route_id = TrafficId::new("route:scenario").expect("stable route ID");
                return rne_traffic::materialize_lane_route(network, &lane_route, route_id, false)
                    .map_err(|error| {
                        ScenarioError::Invalid(format!("route materialization: {error}"))
                    });
            }
        }
    }
    Err(ScenarioError::Invalid(format!(
        "no route between any source and sink lane for {:?}",
        kind
    )))
}

fn spawn_distance_m(route: &TrafficRoute, position: Option<[f64; 3]>) -> f64 {
    let Some(position) = position else {
        return 0.0;
    };
    let length_m = route.total_length_m();
    if length_m <= 0.0 {
        return 0.0;
    }
    let samples = (length_m / 0.5).ceil().max(1.0) as u32;
    let mut best_distance_m = 0.0;
    let mut best_squared = f64::INFINITY;
    for index in 0..=samples {
        let distance_m = length_m * f64::from(index) / f64::from(samples);
        let sample = route.sample(distance_m);
        let dx = sample.position_m[0] - position[0];
        let dy = sample.position_m[1] - position[1];
        let dz = sample.position_m[2] - position[2];
        let squared = dx * dx + dy * dy + dz * dz;
        if squared < best_squared {
            best_squared = squared;
            best_distance_m = distance_m;
        }
    }
    best_distance_m
}

fn uuid_for_entity(index: usize) -> u128 {
    0x0001_0000_0000_0000_0000_0000_0000_0000 | (index as u128)
}

type ActionSchedule = BTreeMap<String, Vec<(f64, ScenarioAction)>>;

/// Builds the parallel route used for lane changes, when any exist.
fn parallel_route_for(
    primary: &TrafficRoute,
    document: &ScenarioDocument,
) -> Result<Option<TrafficRoute>, ScenarioError> {
    let offset = document
        .actions
        .iter()
        .filter_map(|action| match action.action {
            ScenarioAction::LaneChange { target_lane_offset } => Some(target_lane_offset),
            _ => None,
        })
        .next();
    let Some(offset) = offset else {
        return Ok(None);
    };
    let path = primary.path_m();
    let mut offset_path = path.to_vec();
    const LANE_WIDTH_M: f64 = 3.5;
    for index in 0..path.len() {
        let (prev, next) = if path.len() == 1 {
            (path[0], path[0])
        } else if index == 0 {
            (path[0], path[1])
        } else if index + 1 == path.len() {
            (path[index - 1], path[index])
        } else {
            (path[index - 1], path[index + 1])
        };
        let dx = next[0] - prev[0];
        let dz = next[2] - prev[2];
        let length = (dx * dx + dz * dz).sqrt();
        if length <= 1e-9 {
            continue;
        }
        let sign = if offset > 0 { 1.0 } else { -1.0 };
        offset_path[index][0] += -dz / length * LANE_WIDTH_M * sign;
        offset_path[index][2] += dx / length * LANE_WIDTH_M * sign;
    }
    let route_id = TrafficId::new("route:scenario:parallel").expect("stable route ID");
    TrafficRoute::new(route_id, offset_path, primary.is_closed())
        .map(Some)
        .map_err(|error| ScenarioError::Invalid(format!("parallel route: {error}")))
}

fn build_action_schedules(document: &ScenarioDocument) -> ActionSchedule {
    let mut schedules: ActionSchedule = BTreeMap::new();
    for action in &document.actions {
        schedules
            .entry(action.entity.clone())
            .or_default()
            .push((action.start_time_s, action.action));
    }
    for entry in schedules.values_mut() {
        entry.sort_by(|left, right| left.0.total_cmp(&right.0));
    }
    schedules
}

fn apply_due_actions(
    world: &mut World,
    schedules: &ActionSchedule,
    sim_time: SimTime,
    parallel_route: Option<&TrafficRoute>,
) {
    let now_s = sim_time.as_seconds().value();
    for (entity_name, steps) in schedules {
        let Some(entity) = world.iter_entities().find_map(|entity_ref| {
            let entity = entity_ref.id();
            let name = world.get::<Name>(entity)?;
            if name.0 == *entity_name {
                Some(entity)
            } else {
                None
            }
        }) else {
            continue;
        };
        for (start_time_s, action) in steps {
            if *start_time_s > now_s {
                break;
            }
            let Some(mut follower) = world.get_mut::<TrafficRouteFollower>(entity) else {
                continue;
            };
            match action {
                ScenarioAction::AbsoluteSpeed { target_m_s } => {
                    follower.desired_speed_m_s = *target_m_s;
                }
                ScenarioAction::LaneChange { .. } => {
                    if let Some(parallel_route) = parallel_route {
                        follower.route_id = parallel_route.id().clone();
                    }
                }
            }
        }
    }
}
