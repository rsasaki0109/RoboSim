//! Deterministic scenario execution over the traffic runtime.
//!
//! The executor turns a [`ScenarioDocument`] and a [`TrafficNetwork`] into a
//! deterministic fixed-step run: it derives one actor-compatible route from the
//! network, spawns the scenario entities as traffic actors on that route, and
//! applies each storyboard speed action at its scheduled simulation time while
//! stepping the kinematic traffic systems.

use crate::{ScenarioAction, ScenarioDocument, ScenarioEntityKind, ScenarioError};
use rne_core::{EpisodeOutcome, RunControl, SimDuration, SimTime};
use rne_ecs::{EntityUuid, Name, World};
use rne_math::Hertz;
use rne_traffic::{
    shortest_lane_route, SignalAspect, SignalProgram, TrafficActor, TrafficActorKind, TrafficId,
    TrafficNetwork, TrafficOwnershipMetrics, TrafficPose, TrafficPoseSource, TrafficRoute,
    TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime, TrafficSignalControl,
    TrafficSignalControls,
};
use serde::{Deserialize, Serialize};
use std::collections::{btree_map::Entry, BTreeMap, BTreeSet};
use uuid::Uuid;

/// Fixed-step execution settings for a scenario.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioRunOptions {
    /// Number of fixed simulation steps.
    pub steps: u64,
    /// Fixed simulation rate in hertz.
    pub hz: f64,
}

/// Named final state of one scenario actor.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioActorResult {
    /// Stable OpenSCENARIO entity name.
    pub name: String,
    /// Stable UUID derived from canonical entity-name order.
    pub stable_uuid: String,
    /// Road-user kind declared by the scenario.
    pub kind: ScenarioEntityKind,
    /// Subsystem that owned the final pose.
    pub pose_source: TrafficPoseSource,
    /// Stable route ID followed at the end of the run.
    pub route_id: String,
    /// Final world position in metres.
    pub final_position_m: [f64; 3],
    /// Final heading around the positive Y axis in radians.
    pub final_heading_rad: f64,
    /// Final route-follower speed in metres per second.
    pub final_speed_m_s: f64,
}

/// Canonical evidence that one scheduled scenario action was applied.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioActionEvidence {
    /// Simulation time declared by the scenario, in seconds.
    pub start_time_s: f64,
    /// Stable target entity name.
    pub entity_name: String,
    /// Zero-based source action index for this entity.
    pub source_action_index: usize,
    /// Fixed step on which the action was applied.
    pub applied_step: u64,
    /// Action payload that was applied.
    pub action: ScenarioAction,
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
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioRunResult {
    /// Stable hash of the ordered actor state after the last step.
    pub stable_hash: u64,
    /// Digest covering canonical final actor states and applied-action evidence.
    pub result_digest: u64,
    /// Red stop-line crossings across the run.
    pub signal_violations: usize,
    /// Overlapping bumper pairs observed across the run.
    pub collisions: usize,
    /// Actor positions after the last step, in actor spawn order.
    pub final_positions_m: Vec<[f64; 3]>,
    /// Named actor states after the last step, in canonical name order.
    #[serde(default)]
    pub final_actors: Vec<ScenarioActorResult>,
    /// Applied actions in `(start_time_s, entity_name, source_action_index)` order.
    pub action_evidence: Vec<ScenarioActionEvidence>,
    /// Scheduled actions due during the run that were not applied.
    pub unapplied_action_count: usize,
    /// Smallest bumper-to-bumper gap observed during the run.
    pub minimum_observed_gap_m: Option<f64>,
    /// Pose ownership counts from the final completed traffic step.
    pub ownership: TrafficOwnershipMetrics,
    /// Length of the derived traffic route in metres.
    pub route_length_m: f64,
    /// Average speed of departed, unfinished actors after the last step.
    pub average_speed_m_s: f64,
    /// Number of steps completed in the final episode.
    pub steps: u64,
}

impl ScenarioRunResult {
    /// Compares deterministic replay evidence across JSON serialization.
    ///
    /// Runtime and result digests remain exact. Diagnostic SI values use the
    /// same nanounit normalization as the result digest so a JSON parser's
    /// sub-nanounit floating-point rounding does not invalidate a replay.
    pub fn replay_matches(&self, recorded: &Self) -> bool {
        self.stable_hash == recorded.stable_hash
            && self.result_digest == recorded.result_digest
            && self.signal_violations == recorded.signal_violations
            && self.collisions == recorded.collisions
            && self.final_positions_m.len() == recorded.final_positions_m.len()
            && self.final_actors.len() == recorded.final_actors.len()
            && self.action_evidence.len() == recorded.action_evidence.len()
            && self.unapplied_action_count == recorded.unapplied_action_count
            && self.minimum_observed_gap_m.map(quantize_result_value)
                == recorded.minimum_observed_gap_m.map(quantize_result_value)
            && self.ownership == recorded.ownership
            && quantize_result_value(self.route_length_m)
                == quantize_result_value(recorded.route_length_m)
            && quantize_result_value(self.average_speed_m_s)
                == quantize_result_value(recorded.average_speed_m_s)
            && self.steps == recorded.steps
    }
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

/// Mutable state for one deterministic scenario episode.
struct ScenarioEpisode {
    world: World,
    route: TrafficRoute,
    parallel_routes: BTreeMap<(TrafficId, i64), TrafficId>,
    routes: TrafficRouteCatalog,
    runtime: TrafficRuntime,
    schedules: ActionSchedule,
    controls: TrafficSignalControls,
    signal_schedule: Vec<SignalSchedule>,
    action_progress: ActionProgress,
    signal_violations: usize,
    collisions: usize,
    stable_hash: u64,
    average_speed_m_s: f64,
    minimum_observed_gap_m: Option<f64>,
    ownership: TrafficOwnershipMetrics,
}

#[derive(Serialize)]
struct ScenarioLiveSnapshot {
    positions_m: Vec<[f64; 3]>,
    actors: Vec<ScenarioActorResult>,
    signal_violations: usize,
    collisions: usize,
    stable_hash: u64,
    average_speed_m_s: f64,
}

impl ScenarioEpisode {
    fn build(
        document: &ScenarioDocument,
        network: &TrafficNetwork,
        primary_kind: ScenarioEntityKind,
    ) -> Result<Self, ScenarioError> {
        let mut kind_routes = BTreeMap::new();
        for entity in &document.entities {
            if let Entry::Vacant(entry) = kind_routes.entry(entity.kind) {
                let route = derive_route(network, entity.kind, route_id_for_kind(entity.kind))?;
                entry.insert(route);
            }
        }
        let route = kind_routes
            .get(&primary_kind)
            .expect("primary kind was collected from the document")
            .clone();

        let mut world = World::new();
        let mut spawn_order = document.entities.clone();
        spawn_order.sort_by(|left, right| left.name.cmp(&right.name));
        for (index, entity) in spawn_order.iter().enumerate() {
            let entity_route = kind_routes
                .get(&entity.kind)
                .expect("all entity kinds have a route");
            let distance_m = spawn_distance_m(entity_route, entity.initial_world_position_m);
            let pose = entity_route.sample(distance_m);
            world.spawn((
                Name(entity.name.clone()),
                TrafficActor {
                    kind: actor_kind(entity.kind),
                },
                EntityUuid(Uuid::from_u128(uuid_for_entity(index))),
                TrafficRouteFollower {
                    route_id: entity_route.id().clone(),
                    distance_m,
                    speed_m_s: 0.0,
                    desired_speed_m_s: 0.0,
                    length_m: actor_length_m(entity.kind),
                },
                TrafficPose {
                    position_m: pose.position_m,
                    yaw_rad: pose.yaw_rad,
                },
                TrafficPoseSource::Runtime,
            ));
        }

        let mut routes = TrafficRouteCatalog::default();
        for kind_route in kind_routes.values() {
            routes
                .insert(kind_route.clone())
                .map_err(|error| ScenarioError::Invalid(format!("insert route: {error}")))?;
        }
        let mut parallel_routes = BTreeMap::new();
        let offsets = document
            .actions
            .iter()
            .filter_map(|action| match action.action {
                ScenarioAction::LaneChange { target_lane_offset } => Some(target_lane_offset),
                _ => None,
            })
            .collect::<BTreeSet<_>>();
        for kind_route in kind_routes.values() {
            for offset in &offsets {
                let parallel_route = parallel_route_for(kind_route, *offset)?;
                parallel_routes.insert(
                    (kind_route.id().clone(), *offset),
                    parallel_route.id().clone(),
                );
                routes.insert(parallel_route).map_err(|error| {
                    ScenarioError::Invalid(format!("insert parallel route: {error}"))
                })?;
            }
        }
        let (controls, signal_schedule) = build_signal_schedule(network, &route)?;
        Ok(Self {
            world,
            route,
            parallel_routes,
            routes,
            runtime: TrafficRuntime::default(),
            schedules: build_action_schedules(document),
            controls,
            signal_schedule,
            action_progress: ActionProgress::default(),
            signal_violations: 0,
            collisions: 0,
            stable_hash: 0,
            average_speed_m_s: 0.0,
            minimum_observed_gap_m: None,
            ownership: TrafficOwnershipMetrics::default(),
        })
    }

    fn live_snapshot(&self) -> String {
        let actors = collect_actor_results(&self.world);
        serde_json::to_string(&ScenarioLiveSnapshot {
            positions_m: actors.iter().map(|actor| actor.final_position_m).collect(),
            actors,
            signal_violations: self.signal_violations,
            collisions: self.collisions,
            stable_hash: self.stable_hash,
            average_speed_m_s: self.average_speed_m_s,
        })
        .unwrap_or_else(|_| "{}".to_string())
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
    execute_scenario_with_control(document, network, options, None)
}

/// Executes a scenario with optional pause, step, reset, quit, and live-status control.
///
/// When control is None, this has the same deterministic fixed-step behavior
/// as execute_scenario. When present, the control state machine is consulted
/// before every step. Reset rebuilds the episode from its initial conditions,
/// quit returns the current partial episode, and completed steps report a
/// compact JSON snapshot containing actor positions and traffic metrics.
pub fn execute_scenario_with_control(
    document: &ScenarioDocument,
    network: &TrafficNetwork,
    options: &ScenarioRunOptions,
    mut control: Option<&mut RunControl<'_>>,
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
    let delta = SimDuration::from_hertz(Hertz::new(options.hz));
    'episode: loop {
        let mut episode = ScenarioEpisode::build(document, network, primary_kind)?;
        let mut completed_steps = 0;
        for step in 1..=options.steps {
            if let Some(control) = control.as_deref_mut() {
                match control.checkpoint() {
                    EpisodeOutcome::Advance => {}
                    EpisodeOutcome::Reset => continue 'episode,
                    EpisodeOutcome::Quit => break,
                }
            }
            let sim_time = SimTime::from_ticks(step * delta.ticks());
            apply_due_actions(
                &mut episode.world,
                &mut episode.routes,
                &episode.schedules,
                sim_time,
                step,
                &episode.parallel_routes,
                &mut episode.action_progress,
            )?;
            apply_signal_cycle(&mut episode.controls, &episode.signal_schedule, sim_time);
            let report = rne_traffic::advance_controlled_kinematic_traffic(
                &mut episode.world,
                &episode.routes,
                &episode.controls,
                &mut episode.runtime,
                sim_time,
                delta,
                rne_traffic::KinematicTrafficConfig::default(),
            )
            .map_err(|error| ScenarioError::Invalid(format!("traffic step: {error}")))?;
            episode.signal_violations += report.signal_violation_count;
            episode.collisions += report.collision_count;
            episode.stable_hash = report.stable_state_hash;
            episode.average_speed_m_s = report.flow.average_speed_m_s;
            episode.minimum_observed_gap_m = minimum_gap(
                episode.minimum_observed_gap_m,
                report.minimum_observed_gap_m,
            );
            episode.ownership = report.ownership;
            completed_steps = step;
            if let Some(control) = control.as_deref_mut() {
                let snapshot = episode.live_snapshot();
                control.report_status(step, sim_time.as_seconds().value(), snapshot.as_bytes());
            }
        }

        let final_actors = collect_actor_results(&episode.world);
        let final_positions_m = final_actors
            .iter()
            .map(|actor| actor.final_position_m)
            .collect();
        let result_digest =
            scenario_result_digest(&final_actors, &episode.action_progress.evidence)?;
        return Ok(ScenarioRunResult {
            stable_hash: episode.stable_hash,
            result_digest,
            signal_violations: episode.signal_violations,
            collisions: episode.collisions,
            final_positions_m,
            final_actors,
            action_evidence: episode.action_progress.evidence,
            unapplied_action_count: episode
                .schedules
                .len()
                .saturating_sub(episode.action_progress.applied_count),
            minimum_observed_gap_m: episode.minimum_observed_gap_m.map(quantize_result_value),
            ownership: episode.ownership,
            route_length_m: episode.route.total_length_m(),
            average_speed_m_s: episode.average_speed_m_s,
            steps: completed_steps,
        });
    }
}

fn collect_actor_results(world: &World) -> Vec<ScenarioActorResult> {
    let mut actors = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let name = world.get::<Name>(entity)?;
            let uuid = world.get::<EntityUuid>(entity)?;
            let pose = world.get::<TrafficPose>(entity)?;
            let actor = world.get::<TrafficActor>(entity)?;
            let follower = world.get::<TrafficRouteFollower>(entity)?;
            Some(ScenarioActorResult {
                name: name.0.clone(),
                stable_uuid: uuid.0.to_string(),
                kind: scenario_kind(actor.kind),
                pose_source: world
                    .get::<TrafficPoseSource>(entity)
                    .copied()
                    .unwrap_or(TrafficPoseSource::Runtime),
                route_id: follower.route_id.as_str().to_string(),
                final_position_m: pose.position_m,
                final_heading_rad: pose.yaw_rad,
                final_speed_m_s: follower.speed_m_s,
            })
        })
        .collect::<Vec<_>>();
    actors.sort_by(|left, right| left.stable_uuid.cmp(&right.stable_uuid));
    actors
}

fn minimum_gap(current: Option<f64>, observed: Option<f64>) -> Option<f64> {
    match (current, observed) {
        (Some(current), Some(observed)) => Some(current.min(observed)),
        (Some(current), None) => Some(current),
        (None, observed) => observed,
    }
}

fn quantize_result_value(value: f64) -> f64 {
    const RESULT_UNITS_PER_SI: f64 = 1_000_000_000.0;
    (value * RESULT_UNITS_PER_SI).round() / RESULT_UNITS_PER_SI
}

pub(crate) fn scenario_result_digest(
    actors: &[ScenarioActorResult],
    actions: &[ScenarioActionEvidence],
) -> Result<u64, ScenarioError> {
    let mut bytes = b"rne-scenario-result-v1".to_vec();
    push_usize(&mut bytes, actors.len());
    for actor in actors {
        push_string(&mut bytes, &actor.name);
        push_string(&mut bytes, &actor.stable_uuid);
        bytes.push(match actor.kind {
            ScenarioEntityKind::MotorVehicle => 0,
            ScenarioEntityKind::Bicycle => 1,
            ScenarioEntityKind::Pedestrian => 2,
        });
        bytes.push(match actor.pose_source {
            TrafficPoseSource::Runtime => 0,
            TrafficPoseSource::External => 1,
        });
        push_string(&mut bytes, &actor.route_id);
        for value in actor.final_position_m {
            push_quantized_f64(&mut bytes, value)?;
        }
        push_quantized_f64(&mut bytes, actor.final_heading_rad)?;
        push_quantized_f64(&mut bytes, actor.final_speed_m_s)?;
    }
    push_usize(&mut bytes, actions.len());
    for evidence in actions {
        push_quantized_f64(&mut bytes, evidence.start_time_s)?;
        push_string(&mut bytes, &evidence.entity_name);
        push_usize(&mut bytes, evidence.source_action_index);
        bytes.extend_from_slice(&evidence.applied_step.to_le_bytes());
        match &evidence.action {
            ScenarioAction::AbsoluteSpeed { target_m_s } => {
                bytes.push(0);
                push_quantized_f64(&mut bytes, *target_m_s)?;
            }
            ScenarioAction::LaneChange { target_lane_offset } => {
                bytes.push(1);
                bytes.extend_from_slice(&target_lane_offset.to_le_bytes());
            }
            ScenarioAction::AssignRoute { waypoints } => {
                bytes.push(2);
                push_usize(&mut bytes, waypoints.len());
                for waypoint in waypoints {
                    for value in waypoint {
                        push_quantized_f64(&mut bytes, *value)?;
                    }
                }
            }
        }
    }
    Ok(crate::stable_replay_input_digest(&bytes))
}

fn push_string(bytes: &mut Vec<u8>, value: &str) {
    push_usize(bytes, value.len());
    bytes.extend_from_slice(value.as_bytes());
}

fn push_usize(bytes: &mut Vec<u8>, value: usize) {
    bytes.extend_from_slice(&(value as u64).to_le_bytes());
}

fn push_quantized_f64(bytes: &mut Vec<u8>, value: f64) -> Result<(), ScenarioError> {
    const DIGEST_UNITS_PER_SI: f64 = 1_000_000_000.0;
    if !value.is_finite()
        || value < i64::MIN as f64 / DIGEST_UNITS_PER_SI
        || value > i64::MAX as f64 / DIGEST_UNITS_PER_SI
    {
        return Err(ScenarioError::Invalid(
            "result evidence contains a non-finite or out-of-range value".to_string(),
        ));
    }
    let quantized = (value * DIGEST_UNITS_PER_SI).round() as i64;
    bytes.extend_from_slice(&quantized.to_le_bytes());
    Ok(())
}

fn derive_route(
    network: &TrafficNetwork,
    kind: ScenarioEntityKind,
    route_id: TrafficId,
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

fn route_id_for_kind(kind: ScenarioEntityKind) -> TrafficId {
    let suffix = match kind {
        ScenarioEntityKind::MotorVehicle => "motor_vehicle",
        ScenarioEntityKind::Bicycle => "bicycle",
        ScenarioEntityKind::Pedestrian => "pedestrian",
    };
    TrafficId::new(format!("route:scenario:{suffix}")).expect("stable route ID")
}

fn scenario_kind(kind: TrafficActorKind) -> ScenarioEntityKind {
    match kind {
        TrafficActorKind::MotorVehicle => ScenarioEntityKind::MotorVehicle,
        TrafficActorKind::Bicycle => ScenarioEntityKind::Bicycle,
        TrafficActorKind::Pedestrian => ScenarioEntityKind::Pedestrian,
    }
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

#[derive(Clone, Debug)]
struct ScheduledAction {
    start_time_s: f64,
    entity_name: String,
    source_action_index: usize,
    action: ScenarioAction,
}

type ActionSchedule = Vec<ScheduledAction>;

#[derive(Debug, Default)]
struct ActionProgress {
    applied_count: usize,
    evidence: Vec<ScenarioActionEvidence>,
}

/// Builds the parallel route used for lane changes, when any exist.
fn parallel_route_for(primary: &TrafficRoute, offset: i64) -> Result<TrafficRoute, ScenarioError> {
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
    let side = if offset > 0 { "left" } else { "right" };
    let route_id = TrafficId::new(format!("{}:parallel:{side}", primary.id().as_str()))
        .expect("stable parallel route ID");
    TrafficRoute::new(route_id, offset_path, primary.is_closed())
        .map_err(|error| ScenarioError::Invalid(format!("parallel route: {error}")))
}

/// One network signal's fixed-time program tied to a route stop-line control.
struct SignalSchedule {
    /// Control id in the [`TrafficSignalControls`].
    control_id: TrafficId,
    /// Signal-group id whose aspect the control follows.
    group_id: TrafficId,
    /// Deterministic fixed-time program advanced by the run clock.
    program: SignalProgram,
}

/// Derives stop-line controls and cyclic aspects from the network's signals.
///
/// Only signals whose groups control a connection on the derived route are
/// wired, using the group's nearest route connection as its stop distance.
/// Signals without a fixed-time program are skipped.
fn build_signal_schedule(
    network: &TrafficNetwork,
    route: &TrafficRoute,
) -> Result<(TrafficSignalControls, Vec<SignalSchedule>), ScenarioError> {
    let mut controls = TrafficSignalControls::default();
    let mut schedule = Vec::new();
    for signal in &network.signals {
        let Some(program) = &signal.program else {
            continue;
        };
        for group in &signal.groups {
            let stop_distance_m = group
                .connection_ids
                .iter()
                .filter_map(|connection_id| {
                    route
                        .movements()
                        .iter()
                        .find(|movement| &movement.connection_id == connection_id)
                })
                .map(|movement| movement.entry_distance_m)
                .min_by(f64::total_cmp);
            let Some(stop_distance_m) = stop_distance_m else {
                continue;
            };
            let control_id =
                TrafficId::new(format!("{}:{}", signal.id.as_str(), group.id.as_str()))
                    .expect("stable signal control ID");
            let initial_aspect = program
                .phases
                .first()
                .and_then(|phase| {
                    phase
                        .group_aspects
                        .iter()
                        .find(|aspect| aspect.group_id == group.id)
                        .map(|aspect| aspect.aspect)
                })
                .unwrap_or(SignalAspect::Red);
            controls
                .insert(TrafficSignalControl {
                    id: control_id.clone(),
                    route_id: route.id().clone(),
                    stop_distance_m,
                    aspect: initial_aspect,
                })
                .map_err(|error| {
                    ScenarioError::Invalid(format!("insert signal control: {error}"))
                })?;
            schedule.push(SignalSchedule {
                control_id,
                group_id: group.id.clone(),
                program: program.clone(),
            });
        }
    }
    Ok((controls, schedule))
}

/// Advances each signal's aspect to its active program phase at `sim_time`.
fn apply_signal_cycle(
    controls: &mut TrafficSignalControls,
    schedule: &[SignalSchedule],
    sim_time: SimTime,
) {
    let time_s = sim_time.as_seconds().value();
    for entry in schedule {
        let cycle_s = entry
            .program
            .phases
            .iter()
            .map(|phase| phase.duration_s)
            .sum::<f64>();
        if !cycle_s.is_finite() || cycle_s <= 0.0 {
            continue;
        }
        let mut remaining_s = (time_s - entry.program.offset_s).rem_euclid(cycle_s);
        for phase in &entry.program.phases {
            if remaining_s < phase.duration_s {
                if let Some(aspect) = phase
                    .group_aspects
                    .iter()
                    .find(|aspect| aspect.group_id == entry.group_id)
                {
                    let _ = controls.set_aspect(&entry.control_id, aspect.aspect);
                }
                break;
            }
            remaining_s -= phase.duration_s;
        }
    }
}

fn build_action_schedules(document: &ScenarioDocument) -> ActionSchedule {
    let mut source_indices = BTreeMap::<String, usize>::new();
    let mut schedules = Vec::with_capacity(document.actions.len());
    for action in &document.actions {
        let source_action_index = source_indices.entry(action.entity.clone()).or_default();
        schedules.push(ScheduledAction {
            start_time_s: action.start_time_s,
            entity_name: action.entity.clone(),
            source_action_index: *source_action_index,
            action: action.action.clone(),
        });
        *source_action_index += 1;
    }
    schedules.sort_by(|left, right| {
        left.start_time_s
            .total_cmp(&right.start_time_s)
            .then_with(|| left.entity_name.cmp(&right.entity_name))
            .then_with(|| left.source_action_index.cmp(&right.source_action_index))
    });
    schedules
}

fn apply_due_actions(
    world: &mut World,
    routes: &mut TrafficRouteCatalog,
    schedules: &ActionSchedule,
    sim_time: SimTime,
    step: u64,
    parallel_routes: &BTreeMap<(TrafficId, i64), TrafficId>,
    progress: &mut ActionProgress,
) -> Result<(), ScenarioError> {
    let now_s = sim_time.as_seconds().value();
    while let Some(scheduled) = schedules.get(progress.applied_count) {
        if scheduled.start_time_s > now_s {
            break;
        }
        let entity_name = &scheduled.entity_name;
        let Some(entity) = world.iter_entities().find_map(|entity_ref| {
            let entity = entity_ref.id();
            let name = world.get::<Name>(entity)?;
            if name.0 == *entity_name {
                Some(entity)
            } else {
                None
            }
        }) else {
            return Err(ScenarioError::Invalid(format!(
                "scheduled entity `{entity_name}` is missing from the runtime"
            )));
        };
        match &scheduled.action {
            ScenarioAction::AbsoluteSpeed { target_m_s } => {
                if let Some(mut follower) = world.get_mut::<TrafficRouteFollower>(entity) {
                    follower.desired_speed_m_s = *target_m_s;
                }
            }
            ScenarioAction::LaneChange { target_lane_offset } => {
                let current_route_id = world
                    .get::<TrafficRouteFollower>(entity)
                    .map(|follower| follower.route_id.clone())
                    .ok_or_else(|| {
                        ScenarioError::Invalid(format!(
                            "entity `{entity_name}` is missing a route follower"
                        ))
                    })?;
                let target_route_id = parallel_routes
                    .get(&(current_route_id.clone(), *target_lane_offset))
                    .cloned()
                    .ok_or_else(|| {
                        ScenarioError::Invalid(format!(
                            "entity `{entity_name}` cannot lane-change from route `{current_route_id}`"
                        ))
                    })?;
                if let Some(mut follower) = world.get_mut::<TrafficRouteFollower>(entity) {
                    follower.route_id = target_route_id;
                }
            }
            ScenarioAction::AssignRoute { waypoints } => {
                let route_id = assigned_route_id(entity_name, scheduled.source_action_index);
                if routes.get(&route_id).is_none() {
                    let assigned_route =
                        TrafficRoute::new(route_id.clone(), waypoints.clone(), false).map_err(
                            |error| ScenarioError::Invalid(format!("assigned route: {error}")),
                        )?;
                    routes.insert(assigned_route).map_err(|error| {
                        ScenarioError::Invalid(format!("insert assigned route: {error}"))
                    })?;
                }
                let position = world.get::<TrafficPose>(entity).map(|pose| pose.position_m);
                let distance_m = spawn_distance_m(
                    routes.get(&route_id).expect("inserted assigned route"),
                    position,
                );
                if let Some(mut follower) = world.get_mut::<TrafficRouteFollower>(entity) {
                    follower.route_id = route_id;
                    follower.distance_m = distance_m;
                }
            }
        }
        progress.evidence.push(ScenarioActionEvidence {
            start_time_s: scheduled.start_time_s,
            entity_name: scheduled.entity_name.clone(),
            source_action_index: scheduled.source_action_index,
            applied_step: step,
            action: scheduled.action.clone(),
        });
        progress.applied_count += 1;
    }
    Ok(())
}

fn assigned_route_id(entity_name: &str, action_index: usize) -> TrafficId {
    let encoded_name = entity_name
        .as_bytes()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();
    TrafficId::new(format!(
        "route:scenario:assigned:{encoded_name}:{action_index}"
    ))
    .expect("hex-encoded entity route ID is valid")
}
