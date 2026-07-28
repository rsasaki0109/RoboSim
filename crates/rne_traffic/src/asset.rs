//! Versioned traffic-network asset schema.

use crate::{TrafficActorKind, TrafficAssetError, TrafficId};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Stable schema identifier written to every `.rne.traffic.json` asset.
pub const TRAFFIC_ASSET_SCHEMA: &str = "rne.traffic";

/// Current `.rne.traffic.json` schema version.
pub const TRAFFIC_ASSET_SCHEMA_VERSION: u32 = 1;

/// Root document for deterministic RNE traffic assets.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficAsset {
    /// Schema identifier. Must equal [`TRAFFIC_ASSET_SCHEMA`].
    pub schema: String,
    /// Schema version. Must equal [`TRAFFIC_ASSET_SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Traffic network encoded by this asset.
    pub network: TrafficNetwork,
}

impl TrafficAsset {
    /// Creates an empty schema-v1 asset around a network.
    pub fn new(network: TrafficNetwork) -> Self {
        Self {
            schema: TRAFFIC_ASSET_SCHEMA.into(),
            schema_version: TRAFFIC_ASSET_SCHEMA_VERSION,
            network,
        }
    }

    /// Validates schema, stable IDs, references, units, and signal invariants.
    pub fn validate(&self) -> Result<(), TrafficAssetError> {
        if self.schema != TRAFFIC_ASSET_SCHEMA
            || self.schema_version != TRAFFIC_ASSET_SCHEMA_VERSION
        {
            return Err(TrafficAssetError::UnsupportedSchema {
                schema: self.schema.clone(),
                version: self.schema_version,
            });
        }
        self.network.validate()
    }

    /// Returns a clone with every set-like collection in canonical order.
    pub fn canonicalized(&self) -> Self {
        let mut canonical = self.clone();
        canonical.network.canonicalize();
        canonical
    }
}

/// One traffic network in a shared RNE coordinate frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficNetwork {
    /// Globally stable network ID.
    pub id: TrafficId,
    /// Source and accuracy classification for the network.
    pub provenance: Provenance,
    /// Coordinate frame used by all metric geometry.
    pub coordinate_frame: CoordinateFrame,
    /// Directed lanes.
    pub lanes: Vec<Lane>,
    /// Intersections, merges, splits, and tile boundaries.
    pub junctions: Vec<Junction>,
    /// Directed lane-to-lane movements.
    pub connections: Vec<TrafficConnection>,
    /// Signal controllers and physical signal placement.
    pub signals: Vec<TrafficSignal>,
}

impl TrafficNetwork {
    fn canonicalize(&mut self) {
        self.provenance.canonicalize();
        normalize_point(&mut self.coordinate_frame.origin_m);
        self.lanes.sort_by(|left, right| left.id.cmp(&right.id));
        for lane in &mut self.lanes {
            lane.canonicalize();
        }
        self.junctions.sort_by(|left, right| left.id.cmp(&right.id));
        for junction in &mut self.junctions {
            junction.provenance.canonicalize();
            normalize_point(&mut junction.center_m);
        }
        self.connections
            .sort_by(|left, right| left.id.cmp(&right.id));
        for connection in &mut self.connections {
            connection.canonicalize();
        }
        self.signals.sort_by(|left, right| left.id.cmp(&right.id));
        for signal in &mut self.signals {
            signal.canonicalize();
        }
    }

    fn validate(&self) -> Result<(), TrafficAssetError> {
        self.provenance.validate("network", &self.id)?;
        validate_nonempty_text(
            "network",
            &self.id,
            "coordinate_frame.frame_id",
            &self.coordinate_frame.frame_id,
        )?;
        validate_point(
            "network",
            &self.id,
            "coordinate_frame.origin_m",
            self.coordinate_frame.origin_m,
        )?;

        let mut all_ids = BTreeMap::new();
        register_id(&mut all_ids, &self.id, "network")?;
        for lane in &self.lanes {
            register_id(&mut all_ids, &lane.id, "lane")?;
        }
        for junction in &self.junctions {
            register_id(&mut all_ids, &junction.id, "junction")?;
        }
        for connection in &self.connections {
            register_id(&mut all_ids, &connection.id, "connection")?;
        }
        for signal in &self.signals {
            register_id(&mut all_ids, &signal.id, "signal")?;
            for group in &signal.groups {
                register_id(&mut all_ids, &group.id, "signal_group")?;
            }
            if let Some(program) = &signal.program {
                for phase in &program.phases {
                    register_id(&mut all_ids, &phase.id, "signal_phase")?;
                }
            }
        }

        let lane_ids: BTreeSet<_> = self.lanes.iter().map(|lane| lane.id.clone()).collect();
        let junction_ids: BTreeSet<_> = self
            .junctions
            .iter()
            .map(|junction| junction.id.clone())
            .collect();
        let connection_ids: BTreeSet<_> = self
            .connections
            .iter()
            .map(|connection| connection.id.clone())
            .collect();
        let group_ids: BTreeSet<_> = self
            .signals
            .iter()
            .flat_map(|signal| signal.groups.iter().map(|group| group.id.clone()))
            .collect();

        for lane in &self.lanes {
            lane.validate()?;
        }
        for junction in &self.junctions {
            junction.validate()?;
        }
        for connection in &self.connections {
            connection.validate(&lane_ids, &junction_ids, &connection_ids, &group_ids)?;
        }
        for signal in &self.signals {
            signal.validate(&junction_ids, &connection_ids)?;
        }
        validate_signal_membership(&self.connections, &self.signals)?;
        validate_symmetric_conflicts(&self.connections)?;
        Ok(())
    }
}

/// Coordinate convention for metric traffic geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AxisConvention {
    /// RNE right-handed coordinates: X right/east, Y up, and forward/north is negative Z.
    RneYUp,
}

/// Shared coordinate-frame metadata.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CoordinateFrame {
    /// Stable application frame name, such as `map`.
    pub frame_id: String,
    /// Axis convention used by all metric positions.
    pub axis_convention: AxisConvention,
    /// Frame origin expressed in parent-frame meters.
    pub origin_m: [f64; 3],
    /// Optional source coordinate reference system identifier.
    pub source_crs: Option<String>,
}

/// Whether a value came directly from data, was derived, or was scenario-authored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityClass {
    /// Directly encoded by a source dataset.
    Authoritative,
    /// Deterministically calculated from source data.
    Derived,
    /// Authored by an RNE scenario rather than the source dataset.
    Synthetic,
}

/// Qualitative accuracy class attached to source or derived values.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccuracyClass {
    /// Produced from a survey or equivalent measured source.
    Surveyed,
    /// Produced by a source dataset's modeled geometry.
    Modeled,
    /// Calculated deterministically from source geometry or semantics.
    Derived,
    /// Estimated by a documented heuristic.
    Heuristic,
    /// Authored for a simulation scenario.
    ScenarioAuthored,
    /// Accuracy is unavailable from the source.
    Unknown,
}

/// Qualitative and optional metric accuracy.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Accuracy {
    /// Qualitative accuracy class.
    pub class: AccuracyClass,
    /// Optional horizontal accuracy or tolerance in meters.
    pub horizontal_m: Option<f64>,
    /// Optional vertical accuracy or tolerance in meters.
    pub vertical_m: Option<f64>,
}

/// Stable reference to one source feature.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceReference {
    /// Dataset name and version.
    pub dataset: String,
    /// Optional stable feature ID within the dataset.
    pub feature_id: Option<String>,
    /// Optional source or license URI.
    pub uri: Option<String>,
}

/// Source authority, accuracy, and derivation metadata.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Provenance {
    /// Direct, derived, or synthetic authority.
    pub authority: AuthorityClass,
    /// Accuracy classification and optional metric tolerances.
    pub accuracy: Accuracy,
    /// Stable source references.
    pub sources: Vec<SourceReference>,
    /// Derivation or scenario-authoring method.
    pub method: Option<String>,
}

impl Provenance {
    fn canonicalize(&mut self) {
        self.sources.sort();
        if let Some(value) = &mut self.accuracy.horizontal_m {
            normalize_number(value);
        }
        if let Some(value) = &mut self.accuracy.vertical_m {
            normalize_number(value);
        }
    }

    fn validate(
        &self,
        owner_kind: &'static str,
        owner_id: &TrafficId,
    ) -> Result<(), TrafficAssetError> {
        if self.authority != AuthorityClass::Synthetic && self.sources.is_empty() {
            return invalid(
                owner_kind,
                owner_id,
                "provenance.sources",
                "authoritative and derived values require at least one source",
            );
        }
        if self.authority == AuthorityClass::Derived
            && self
                .method
                .as_deref()
                .map(str::trim)
                .filter(|method| !method.is_empty())
                .is_none()
        {
            return invalid(
                owner_kind,
                owner_id,
                "provenance.method",
                "derived values require a non-empty method",
            );
        }
        for source in &self.sources {
            validate_nonempty_text(
                owner_kind,
                owner_id,
                "provenance.sources.dataset",
                &source.dataset,
            )?;
            if let Some(feature_id) = &source.feature_id {
                validate_nonempty_text(
                    owner_kind,
                    owner_id,
                    "provenance.sources.feature_id",
                    feature_id,
                )?;
            }
            if let Some(uri) = &source.uri {
                validate_nonempty_text(owner_kind, owner_id, "provenance.sources.uri", uri)?;
            }
        }
        if let Some(method) = &self.method {
            validate_nonempty_text(owner_kind, owner_id, "provenance.method", method)?;
        }
        validate_optional_nonnegative(
            owner_kind,
            owner_id,
            "provenance.accuracy.horizontal_m",
            self.accuracy.horizontal_m,
        )?;
        validate_optional_nonnegative(
            owner_kind,
            owner_id,
            "provenance.accuracy.vertical_m",
            self.accuracy.vertical_m,
        )
    }
}

/// Semantic lane type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneKind {
    /// Motor-vehicle carriageway lane.
    Driving,
    /// Bicycle lane or track.
    Bicycle,
    /// Pedestrian sidewalk or footway.
    Sidewalk,
    /// Road shoulder.
    Shoulder,
    /// Median or traffic island not used for travel.
    Median,
    /// Parking lane or bay.
    Parking,
    /// Source semantics are not known.
    Unknown,
}

/// Directed traffic lane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lane {
    /// Globally stable lane ID.
    pub id: TrafficId,
    /// Source and accuracy classification.
    pub provenance: Provenance,
    /// Lane semantic type.
    pub kind: LaneKind,
    /// Allowed road-user classes.
    pub allowed_actors: Vec<TrafficActorKind>,
    /// Directed centerline points in frame meters.
    pub centerline_m: Vec<[f64; 3]>,
    /// Nominal lane width in meters.
    pub width_m: f64,
    /// Optional speed limit in meters per second.
    pub speed_limit_m_s: Option<f64>,
    /// Optional source road class code.
    pub road_class: Option<String>,
    /// Source road function codes.
    pub road_functions: Vec<String>,
}

impl Lane {
    fn canonicalize(&mut self) {
        self.provenance.canonicalize();
        self.allowed_actors
            .sort_by_key(|actor| actor_sort_key(*actor));
        self.allowed_actors.dedup();
        for point in &mut self.centerline_m {
            normalize_point(point);
        }
        normalize_number(&mut self.width_m);
        if let Some(speed) = &mut self.speed_limit_m_s {
            normalize_number(speed);
        }
        self.road_functions.sort();
        self.road_functions.dedup();
    }

    fn validate(&self) -> Result<(), TrafficAssetError> {
        self.provenance.validate("lane", &self.id)?;
        if self.centerline_m.len() < 2 {
            return invalid(
                "lane",
                &self.id,
                "centerline_m",
                "a directed lane requires at least two points",
            );
        }
        for point in &self.centerline_m {
            validate_point("lane", &self.id, "centerline_m", *point)?;
        }
        validate_positive("lane", &self.id, "width_m", self.width_m)?;
        if let Some(speed) = self.speed_limit_m_s {
            validate_positive("lane", &self.id, "speed_limit_m_s", speed)?;
        }
        if self.allowed_actors.is_empty() && self.kind != LaneKind::Median {
            return invalid(
                "lane",
                &self.id,
                "allowed_actors",
                "travel lanes require at least one actor kind",
            );
        }
        if let Some(road_class) = &self.road_class {
            validate_nonempty_text("lane", &self.id, "road_class", road_class)?;
        }
        for road_function in &self.road_functions {
            validate_nonempty_text("lane", &self.id, "road_functions", road_function)?;
        }
        Ok(())
    }
}

/// Junction topology class.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JunctionKind {
    /// T-shaped intersection.
    TIntersection,
    /// Four-way cross intersection.
    CrossIntersection,
    /// Other at-grade intersection.
    Intersection,
    /// Lane merge.
    Merge,
    /// Lane split.
    Split,
    /// Roundabout.
    Roundabout,
    /// Deterministic connection point across asset tile boundaries.
    TileBoundary,
    /// Source semantics are not known.
    Unknown,
}

/// Intersection or lane-connection anchor.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Junction {
    /// Globally stable junction ID.
    pub id: TrafficId,
    /// Source and accuracy classification.
    pub provenance: Provenance,
    /// Junction topology class.
    pub kind: JunctionKind,
    /// Representative center in frame meters.
    pub center_m: [f64; 3],
}

impl Junction {
    fn validate(&self) -> Result<(), TrafficAssetError> {
        self.provenance.validate("junction", &self.id)?;
        validate_point("junction", &self.id, "center_m", self.center_m)
    }
}

/// Connection movement relative to the incoming lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MovementKind {
    /// Continue approximately straight.
    Straight,
    /// Turn left.
    Left,
    /// Turn right.
    Right,
    /// Reverse direction.
    UTurn,
    /// Merge into another lane.
    Merge,
    /// Split away from another lane.
    Split,
}

/// Directed movement from one lane to another.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficConnection {
    /// Globally stable connection ID.
    pub id: TrafficId,
    /// Source and accuracy classification.
    pub provenance: Provenance,
    /// Incoming directed lane.
    pub incoming_lane_id: TrafficId,
    /// Outgoing directed lane.
    pub outgoing_lane_id: TrafficId,
    /// Optional junction containing the movement.
    pub junction_id: Option<TrafficId>,
    /// Relative movement class.
    pub movement: MovementKind,
    /// Directed turn or continuation path in frame meters.
    pub path_m: Vec<[f64; 3]>,
    /// Other connection IDs whose swept paths conflict with this movement.
    pub conflict_connection_ids: Vec<TrafficId>,
    /// Optional signal group controlling this movement.
    pub signal_group_id: Option<TrafficId>,
}

impl TrafficConnection {
    fn canonicalize(&mut self) {
        self.provenance.canonicalize();
        for point in &mut self.path_m {
            normalize_point(point);
        }
        self.conflict_connection_ids.sort();
        self.conflict_connection_ids.dedup();
    }

    fn validate(
        &self,
        lane_ids: &BTreeSet<TrafficId>,
        junction_ids: &BTreeSet<TrafficId>,
        connection_ids: &BTreeSet<TrafficId>,
        group_ids: &BTreeSet<TrafficId>,
    ) -> Result<(), TrafficAssetError> {
        self.provenance.validate("connection", &self.id)?;
        require_reference(
            "connection",
            &self.id,
            "lane",
            &self.incoming_lane_id,
            lane_ids,
        )?;
        require_reference(
            "connection",
            &self.id,
            "lane",
            &self.outgoing_lane_id,
            lane_ids,
        )?;
        if let Some(junction_id) = &self.junction_id {
            require_reference(
                "connection",
                &self.id,
                "junction",
                junction_id,
                junction_ids,
            )?;
        }
        if self.path_m.len() < 2 {
            return invalid(
                "connection",
                &self.id,
                "path_m",
                "a movement path requires at least two points",
            );
        }
        for point in &self.path_m {
            validate_point("connection", &self.id, "path_m", *point)?;
        }
        for conflict_id in &self.conflict_connection_ids {
            require_reference(
                "connection",
                &self.id,
                "connection",
                conflict_id,
                connection_ids,
            )?;
            if conflict_id == &self.id {
                return invalid(
                    "connection",
                    &self.id,
                    "conflict_connection_ids",
                    "a connection cannot conflict with itself",
                );
            }
        }
        if let Some(group_id) = &self.signal_group_id {
            require_reference("connection", &self.id, "signal_group", group_id, group_ids)?;
        }
        Ok(())
    }
}

/// Signal aspect presented to a controlled movement.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalAspect {
    /// Movement must stop.
    Red,
    /// Movement should stop when safe.
    Yellow,
    /// Movement may proceed subject to conflict rules.
    Green,
    /// Signal is not operating.
    Off,
}

/// Set of connections that always share one signal aspect.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalGroup {
    /// Globally stable signal-group ID.
    pub id: TrafficId,
    /// Connections controlled by the group.
    pub connection_ids: Vec<TrafficId>,
}

/// One signal group's aspect during a phase.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalGroupAspect {
    /// Referenced signal-group ID.
    pub group_id: TrafficId,
    /// Aspect active for the phase.
    pub aspect: SignalAspect,
}

/// Fixed-duration signal phase.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalPhase {
    /// Globally stable phase ID.
    pub id: TrafficId,
    /// Phase duration in simulation seconds.
    pub duration_s: f64,
    /// Aspect for every group owned by the signal.
    pub group_aspects: Vec<SignalGroupAspect>,
}

/// Deterministic cyclic fixed-time signal program.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalProgram {
    /// Provenance of the timing program, independently of physical signal geometry.
    pub provenance: Provenance,
    /// Cycle offset in simulation seconds.
    pub offset_s: f64,
    /// Ordered phases. Phase order is semantic and is never canonicalized by ID.
    pub phases: Vec<SignalPhase>,
}

/// Signal controller with optional physical placement and timing program.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficSignal {
    /// Globally stable signal ID.
    pub id: TrafficId,
    /// Provenance of the controller and physical placement.
    pub provenance: Provenance,
    /// Optional junction controlled by this signal.
    pub junction_id: Option<TrafficId>,
    /// Optional physical signal position in frame meters.
    pub position_m: Option<[f64; 3]>,
    /// Optional Y-up facing yaw in radians.
    pub facing_yaw_rad: Option<f64>,
    /// Signal groups controlled together.
    pub groups: Vec<SignalGroup>,
    /// Optional fixed-time program.
    pub program: Option<SignalProgram>,
}

impl TrafficSignal {
    fn canonicalize(&mut self) {
        self.provenance.canonicalize();
        if let Some(position) = &mut self.position_m {
            normalize_point(position);
        }
        if let Some(yaw) = &mut self.facing_yaw_rad {
            normalize_number(yaw);
        }
        self.groups.sort_by(|left, right| left.id.cmp(&right.id));
        for group in &mut self.groups {
            group.connection_ids.sort();
            group.connection_ids.dedup();
        }
        if let Some(program) = &mut self.program {
            program.provenance.canonicalize();
            normalize_number(&mut program.offset_s);
            for phase in &mut program.phases {
                normalize_number(&mut phase.duration_s);
                phase
                    .group_aspects
                    .sort_by(|left, right| left.group_id.cmp(&right.group_id));
            }
        }
    }

    fn validate(
        &self,
        junction_ids: &BTreeSet<TrafficId>,
        connection_ids: &BTreeSet<TrafficId>,
    ) -> Result<(), TrafficAssetError> {
        self.provenance.validate("signal", &self.id)?;
        if let Some(junction_id) = &self.junction_id {
            require_reference("signal", &self.id, "junction", junction_id, junction_ids)?;
        }
        if let Some(position) = self.position_m {
            validate_point("signal", &self.id, "position_m", position)?;
        }
        if let Some(yaw) = self.facing_yaw_rad {
            validate_finite("signal", &self.id, "facing_yaw_rad", yaw)?;
        }
        if self.groups.is_empty() {
            return invalid(
                "signal",
                &self.id,
                "groups",
                "a signal requires at least one group",
            );
        }
        let group_ids: BTreeSet<_> = self.groups.iter().map(|group| group.id.clone()).collect();
        for group in &self.groups {
            if group.connection_ids.is_empty() {
                return invalid(
                    "signal_group",
                    &group.id,
                    "connection_ids",
                    "a signal group requires at least one controlled connection",
                );
            }
            for connection_id in &group.connection_ids {
                require_reference(
                    "signal_group",
                    &group.id,
                    "connection",
                    connection_id,
                    connection_ids,
                )?;
            }
        }
        if let Some(program) = &self.program {
            program.provenance.validate("signal_program", &self.id)?;
            if !program.offset_s.is_finite() || program.offset_s < 0.0 {
                return invalid(
                    "signal",
                    &self.id,
                    "program.offset_s",
                    "offset must be finite and non-negative",
                );
            }
            if program.phases.is_empty() {
                return invalid(
                    "signal",
                    &self.id,
                    "program.phases",
                    "a signal program requires at least one phase",
                );
            }
            for phase in &program.phases {
                validate_positive("signal_phase", &phase.id, "duration_s", phase.duration_s)?;
                let aspect_ids: BTreeSet<_> = phase
                    .group_aspects
                    .iter()
                    .map(|state| state.group_id.clone())
                    .collect();
                if aspect_ids.len() != phase.group_aspects.len() || aspect_ids != group_ids {
                    return invalid(
                        "signal_phase",
                        &phase.id,
                        "group_aspects",
                        "each signal group must appear exactly once",
                    );
                }
            }
        }
        Ok(())
    }
}

fn validate_signal_membership(
    connections: &[TrafficConnection],
    signals: &[TrafficSignal],
) -> Result<(), TrafficAssetError> {
    let groups: BTreeMap<_, _> = signals
        .iter()
        .flat_map(|signal| signal.groups.iter())
        .map(|group| (group.id.clone(), group))
        .collect();
    let connections_by_id: BTreeMap<_, _> = connections
        .iter()
        .map(|connection| (connection.id.clone(), connection))
        .collect();

    for connection in connections {
        if let Some(group_id) = &connection.signal_group_id {
            let group = &groups[group_id];
            if !group.connection_ids.contains(&connection.id) {
                return invalid(
                    "connection",
                    &connection.id,
                    "signal_group_id",
                    "referenced group does not contain this connection",
                );
            }
        }
    }
    for group in groups.values() {
        for connection_id in &group.connection_ids {
            let connection = connections_by_id[connection_id];
            if connection.signal_group_id.as_ref() != Some(&group.id) {
                return invalid(
                    "signal_group",
                    &group.id,
                    "connection_ids",
                    "controlled connection does not reference this group",
                );
            }
        }
    }
    Ok(())
}

fn validate_symmetric_conflicts(
    connections: &[TrafficConnection],
) -> Result<(), TrafficAssetError> {
    let by_id: BTreeMap<_, _> = connections
        .iter()
        .map(|connection| (connection.id.clone(), connection))
        .collect();
    for connection in connections {
        for conflict_id in &connection.conflict_connection_ids {
            if !by_id[conflict_id]
                .conflict_connection_ids
                .contains(&connection.id)
            {
                return invalid(
                    "connection",
                    &connection.id,
                    "conflict_connection_ids",
                    format!("conflict with `{conflict_id}` is not symmetric"),
                );
            }
        }
    }
    Ok(())
}

fn register_id(
    ids: &mut BTreeMap<TrafficId, &'static str>,
    id: &TrafficId,
    kind: &'static str,
) -> Result<(), TrafficAssetError> {
    if let Some(first_kind) = ids.insert(id.clone(), kind) {
        return Err(TrafficAssetError::DuplicateId {
            id: id.clone(),
            first_kind,
            second_kind: kind,
        });
    }
    Ok(())
}

fn require_reference(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    target_kind: &'static str,
    target_id: &TrafficId,
    valid_ids: &BTreeSet<TrafficId>,
) -> Result<(), TrafficAssetError> {
    if valid_ids.contains(target_id) {
        Ok(())
    } else {
        Err(TrafficAssetError::MissingReference {
            owner_kind,
            owner_id: owner_id.clone(),
            target_kind,
            target_id: target_id.clone(),
        })
    }
}

fn validate_nonempty_text(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    field: &'static str,
    value: &str,
) -> Result<(), TrafficAssetError> {
    if value.trim().is_empty() {
        invalid(owner_kind, owner_id, field, "value must not be empty")
    } else {
        Ok(())
    }
}

fn validate_point(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    field: &'static str,
    point: [f64; 3],
) -> Result<(), TrafficAssetError> {
    if point.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        invalid(owner_kind, owner_id, field, "coordinates must be finite")
    }
}

fn validate_finite(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    field: &'static str,
    value: f64,
) -> Result<(), TrafficAssetError> {
    if value.is_finite() {
        Ok(())
    } else {
        invalid(owner_kind, owner_id, field, "value must be finite")
    }
}

fn validate_positive(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    field: &'static str,
    value: f64,
) -> Result<(), TrafficAssetError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        invalid(
            owner_kind,
            owner_id,
            field,
            "value must be finite and greater than zero",
        )
    }
}

fn validate_optional_nonnegative(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    field: &'static str,
    value: Option<f64>,
) -> Result<(), TrafficAssetError> {
    if value
        .map(|value| value.is_finite() && value >= 0.0)
        .unwrap_or(true)
    {
        Ok(())
    } else {
        invalid(
            owner_kind,
            owner_id,
            field,
            "value must be finite and non-negative",
        )
    }
}

fn invalid(
    owner_kind: &'static str,
    owner_id: &TrafficId,
    field: &'static str,
    message: impl Into<String>,
) -> Result<(), TrafficAssetError> {
    Err(TrafficAssetError::InvalidValue {
        owner_kind,
        owner_id: owner_id.clone(),
        field,
        message: message.into(),
    })
}

fn normalize_point(point: &mut [f64; 3]) {
    for value in point {
        normalize_number(value);
    }
}

fn normalize_number(value: &mut f64) {
    if *value == 0.0 {
        *value = 0.0;
    }
}

fn actor_sort_key(actor: TrafficActorKind) -> u8 {
    match actor {
        TrafficActorKind::MotorVehicle => 0,
        TrafficActorKind::Bicycle => 1,
        TrafficActorKind::Pedestrian => 2,
    }
}
