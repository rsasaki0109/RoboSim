//! Imports official PLATEAU data and renders physics-aware LiDAR in Sanjo traffic.

use png::{BitDepth, ColorType, Encoder};
use rne_assets::{
    load_scene_bundle, mesh_package_roots, spawn_scene_bundle, SceneAssetBundle,
    SceneCollisionAsset, SpawnSceneOptions,
};
use rne_core::{SimClock, SimDuration, SimTime};
use rne_data::PointCloud;
use rne_ecs::{spawn_named, Entity, EntityUuid, Name, World};
use rne_math::{Hertz, Quat, Transform3 as MathTransform3, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_plateau::{import_citygml_file, CoordinateMode, ImportOptions, ImportedLane, SourceOrigin};
use rne_render::{
    load_mesh_parts, Camera, HeadlessRenderBackend, ImageFrame, RenderBackend, RenderScene,
    RenderSceneItem, TriangleMesh, Visual, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
#[cfg(test)]
use rne_robot::{ackermann_kinematics, command_ackermann_drive};
use rne_robot::{pure_pursuit_steering, vehicle_dynamics, AckermannDrive, VehicleDynamics};
use rne_sensor::{
    sample_camera_rgbd_swept, sample_lidar_swept, CameraDistortion, CameraRgbdSample, CameraSpec,
    CameraSweep, LidarAtmosphere, LidarDomainRandomization, LidarMaterial, LidarSpec, LidarSweep,
    SensorNoiseKey,
};
#[cfg(test)]
use rne_traffic::advance_kinematic_traffic;
use rne_traffic::{
    advance_reserved_kinematic_traffic, build_traffic_topology, load_traffic_asset,
    materialize_lane_route, shortest_lane_route, KinematicTrafficConfig, KinematicTrafficControls,
    LaneRoute, MovementKind, SignalAspect, TopologyBuildConfig, TrafficActor, TrafficActorKind,
    TrafficConflictControls, TrafficDeparture, TrafficFlowMetrics, TrafficId, TrafficNetwork,
    TrafficPose, TrafficRoute, TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime,
    TrafficSignalControl, TrafficSignalControls,
};
use rne_world::{RandomStreamId, Transform3, WorldRandom};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;
use uuid::Uuid;

const WIDTH: u32 = 1_280;
const HEIGHT: u32 = 720;
const CAR_FRAME_COUNT: usize = 144;
const RENDER_HZ: usize = 12;
const SIM_HZ: usize = 60;
const SIM_STEPS_PER_FRAME: usize = SIM_HZ / RENDER_HZ;
const CLEAR_COLOR: [f32; 4] = [0.34, 0.52, 0.70, 1.0];
const MAX_STATIC_SCENE_ITEMS: usize = 400;
const MAX_DEBUG_SCENE_ITEMS: usize = 3_000;
const SANJO_ORIGIN: SourceOrigin = SourceOrigin {
    first_deg_or_m: 37.631_938_029_139_7,
    second_deg_or_m: 138.955_122_347_658_72,
    height_m: 0.0,
};
const KITA_SANJO_STATION_XZ_M: [f64; 2] = [59.46, -77.17];
#[cfg(test)]
const SIGNAL_GREEN_TIME_S: f64 = 7.0;
const CITY_ACTOR_COUNT: usize = 100;
const CITY_REPLAY_STEPS: u64 = 720;
const CITY_SIGNAL_COUNT: usize = 3;
const CITY_ROUTE_COUNT: usize = 8;
const LIDAR_RAY_COUNT: u32 = 720;
/// Elevation channels, matching a 16-beam spinning scanner.
const LIDAR_CHANNEL_COUNT: u16 = 16;
/// Vertical field of view, matching the +/-15 degrees of a VLP-16 class sensor.
const LIDAR_MIN_ELEVATION_RAD: f64 = -0.261_799_387_799_149_44;
const LIDAR_MAX_ELEVATION_RAD: f64 = 0.261_799_387_799_149_44;
/// One revolution per rendered frame, so the sweep spans exactly the platform motion.
const LIDAR_ROTATION_PERIOD_S: f64 = 1.0 / RENDER_HZ as f64;
/// Beam footprint sub-samples used to produce mixed pixels on silhouettes.
const LIDAR_BEAM_SAMPLE_COUNT: u8 = 3;
const LIDAR_SATURATION_INTENSITY: f64 = 0.92;
const LIDAR_MAX_RANGE_M: f64 = 80.0;
const LIDAR_STREAM_ID: u64 = 46_905;
/// Onboard forward camera resolution, sized for the picture-in-picture insets.
const CAMERA_WIDTH: u32 = 320;
const CAMERA_HEIGHT: u32 = 180;
/// Vertical field of view of a typical forward ADAS camera.
const CAMERA_FOV_Y_RAD: f64 = 0.715_584_993_317_675_1;
/// Slight downward tilt so the camera frames the road ahead.
const CAMERA_PITCH_RAD: f64 = -0.045;
const CAMERA_STREAM_ID: u64 = 46_906;
/// Root world seed shared by the deterministic sensor captures.
const CITY_WORLD_SEED: u64 = 46;
/// Sensor readout time; the car covers real ground while the rows are scanned out.
const CAMERA_READOUT_TIME_S: f64 = 0.02;
/// Bands rendered per frame to approximate continuous row-by-row readout.
const CAMERA_ROLLING_SHUTTER_BANDS: u16 = 8;
/// Inset margin and border thickness in pixels.
const CAMERA_INSET_MARGIN_PX: u32 = 24;
const CAMERA_INSET_BORDER_PX: u32 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SignalPhase {
    Red,
    Green,
}

#[cfg(test)]
fn signal_phase_at(time_s: f64) -> SignalPhase {
    if time_s < SIGNAL_GREEN_TIME_S {
        SignalPhase::Red
    } else {
        SignalPhase::Green
    }
}

#[derive(Clone, Debug, PartialEq)]
struct TurnRoute {
    points: Vec<Vec3>,
    intersection: Vec3,
    incoming_direction: Vec3,
    outgoing_direction: Vec3,
    incoming_half_width_m: f64,
    stop_point: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Footprint {
    min_x_m: f64,
    max_x_m: f64,
    min_z_m: f64,
    max_z_m: f64,
}

impl Footprint {
    fn overlaps_disc(self, center: Vec3, radius_m: f64) -> bool {
        let nearest_x_m = center.x.clamp(self.min_x_m, self.max_x_m);
        let nearest_z_m = center.z.clamp(self.min_z_m, self.max_z_m);
        (center.x - nearest_x_m).powi(2) + (center.z - nearest_z_m).powi(2) < radius_m.powi(2)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct StreetFixture {
    center: Vec3,
    clearance_radius_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct RoadFrame {
    center: Vec3,
    direction: Vec3,
    right: Vec3,
    length_m: f64,
    half_width_m: f64,
    yaw_rad: f64,
}

impl RoadFrame {
    fn from_lanes(lanes: &[ImportedLane]) -> Self {
        assert_eq!(lanes.len(), 2, "streetscape requires an opposing lane pair");
        let start = (Vec3::from_array(lanes[0].centerline_m[0])
            + Vec3::from_array(lanes[1].centerline_m[1]))
            * 0.5;
        let end = (Vec3::from_array(lanes[0].centerline_m[1])
            + Vec3::from_array(lanes[1].centerline_m[0]))
            * 0.5;
        let delta = end - start;
        let direction = delta.normalize_or_zero();
        let right = Vec3::new(-direction.z, 0.0, direction.x);
        let lane_midpoints = [
            (Vec3::from_array(lanes[0].centerline_m[0])
                + Vec3::from_array(lanes[0].centerline_m[1]))
                * 0.5,
            (Vec3::from_array(lanes[1].centerline_m[0])
                + Vec3::from_array(lanes[1].centerline_m[1]))
                * 0.5,
        ];
        let lane_separation_m = (lane_midpoints[1] - lane_midpoints[0]).dot(right).abs();
        let average_lane_width_m = (lanes[0].width_m + lanes[1].width_m) * 0.5;
        Self {
            center: (start + end) * 0.5,
            direction,
            right,
            length_m: delta.length(),
            half_width_m: (lane_separation_m + average_lane_width_m) * 0.5,
            yaw_rad: -direction.z.atan2(direction.x),
        }
    }

    fn point(self, along_m: f64, lateral_m: f64, height_m: f64) -> Vec3 {
        self.center
            + self.direction * along_m
            + self.right * lateral_m
            + Vec3::new(0.0, height_m, 0.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct VehicleFrame {
    transform: Transform3,
    speed_m_s: f64,
    steering_rad: f64,
    wheel_rotation_rad: f64,
    braking: bool,
    class: CityVehicleClass,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CityReplayResult {
    stable_hash: u64,
    collision_count: usize,
    signal_violation_count: usize,
    minimum_gap_m: f64,
    maximum_active_reservations: usize,
    maximum_queue_length: usize,
    flow: TrafficFlowMetrics,
}

#[derive(Clone, Debug, PartialEq)]
struct CityLidarFrame {
    mount: Transform3,
    cloud: PointCloud,
}

#[derive(Clone, Debug, PartialEq)]
struct CityLidarCapture {
    frames: Vec<CityLidarFrame>,
    stable_hash: u64,
    total_returns: usize,
    multi_returns: usize,
    saturated_returns: usize,
    average_intensity: f64,
}

/// One deterministic onboard-camera observation.
#[derive(Clone, Copy, Debug, PartialEq)]
struct CityCameraFrame {
    center_depth_m: f32,
    min_depth_m: f32,
}

#[derive(Clone, Debug, PartialEq)]
struct CityCameraCapture {
    frames: Vec<CityCameraFrame>,
    stable_hash: u64,
    pixels_per_frame: usize,
    nearest_depth_m: f32,
    mean_center_depth_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum CityVehicleClass {
    Compact,
    Sedan,
    Van,
    Bus,
}

impl CityVehicleClass {
    fn length_m(self) -> f64 {
        match self {
            Self::Compact => 3.7,
            Self::Sedan => 4.4,
            Self::Van => 5.2,
            Self::Bus => 8.5,
        }
    }

    fn render_scale(self) -> Vec3 {
        match self {
            Self::Compact => Vec3::new(1.05, 1.14, 1.46),
            Self::Sedan => Vec3::new(1.24, 1.25, 1.70),
            Self::Van => Vec3::new(1.34, 1.52, 1.92),
            Self::Bus => Vec3::new(1.50, 1.68, 2.48),
        }
    }

    fn width_m(self) -> f64 {
        match self {
            Self::Compact => 1.68,
            Self::Sedan => 1.82,
            Self::Van => 1.98,
            Self::Bus => 2.48,
        }
    }

    fn height_m(self) -> f64 {
        match self {
            Self::Compact => 1.48,
            Self::Sedan => 1.52,
            Self::Van => 2.16,
            Self::Bus => 3.18,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CityActorSpec {
    uuid: Uuid,
    route_id: TrafficId,
    distance_m: f64,
    desired_speed_m_s: f64,
    departure_time_s: f64,
    class: CityVehicleClass,
}

#[derive(Clone, Debug, PartialEq)]
struct CitySignalSpec {
    id: TrafficId,
    route_id: TrafficId,
    stop_distance_m: f64,
    phase_offset_steps: u64,
}

#[derive(Clone, Debug, PartialEq)]
struct CityTrafficScenario {
    lane_routes: Vec<LaneRoute>,
    routes: TrafficRouteCatalog,
    actors: Vec<CityActorSpec>,
    signals: Vec<CitySignalSpec>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TrafficDebugOverlay {
    lanes: bool,
    route: bool,
    signals: bool,
    connections: bool,
    conflict_points: bool,
}

impl TrafficDebugOverlay {
    fn from_environment() -> Self {
        let Ok(value) = std::env::var("RNE_TRAFFIC_DEBUG") else {
            return Self::default();
        };
        let mut overlay = Self::default();
        for token in value.split(',').map(str::trim) {
            match token {
                "all" => {
                    overlay = Self {
                        lanes: true,
                        route: true,
                        signals: true,
                        connections: true,
                        conflict_points: true,
                    };
                }
                "lanes" => overlay.lanes = true,
                "route" => overlay.route = true,
                "signals" => overlay.signals = true,
                "connections" => overlay.connections = true,
                "conflicts" | "conflict_points" => overlay.conflict_points = true,
                "none" | "" => {}
                unknown => eprintln!("ignoring unknown RNE_TRAFFIC_DEBUG layer `{unknown}`"),
            }
        }
        overlay
    }
}

#[derive(Clone, Debug)]
struct VehicleRenderAssets {
    body_meshes: Vec<Arc<TriangleMesh>>,
    wheel_meshes: Vec<Arc<TriangleMesh>>,
    red_body_texture: Arc<ImageFrame>,
    blue_body_texture: Arc<ImageFrame>,
    wheel_texture: Arc<ImageFrame>,
}

impl VehicleRenderAssets {
    fn load() -> Self {
        let asset_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/kenney_car");
        Self {
            body_meshes: load_vehicle_meshes(&asset_dir.join("sedan-body.obj")),
            wheel_meshes: load_vehicle_meshes(&asset_dir.join("wheel.obj")),
            red_body_texture: load_vehicle_texture(&asset_dir.join("colormap-red.png")),
            blue_body_texture: load_vehicle_texture(&asset_dir.join("colormap-blue.png")),
            wheel_texture: load_vehicle_texture(&asset_dir.join("colormap.png")),
        }
    }
}

fn load_vehicle_meshes(path: &Path) -> Vec<Arc<TriangleMesh>> {
    let meshes: Vec<_> = load_mesh_parts(path)
        .unwrap_or_else(|error| panic!("load vehicle mesh {}: {error}", path.display()))
        .into_iter()
        .map(|part| Arc::new(part.mesh))
        .collect();
    assert!(!meshes.is_empty(), "vehicle mesh must contain geometry");
    meshes
}

fn load_vehicle_texture(path: &Path) -> Arc<ImageFrame> {
    let rgba = image::open(path)
        .unwrap_or_else(|error| panic!("load vehicle texture {}: {error}", path.display()))
        .into_rgba8();
    Arc::new(ImageFrame::from_rgba8(
        rgba.width(),
        rgba.height(),
        rgba.into_raw(),
    ))
}

#[cfg(test)]
#[derive(Clone, Copy, Debug)]
struct ShowcaseBuilding {
    id: &'static str,
    name: &'static str,
    x_min_m: f64,
    x_max_m: f64,
    z_min_m: f64,
    z_max_m: f64,
    height_m: f64,
}

#[cfg(test)]
const SHOWCASE_BUILDINGS: [ShowcaseBuilding; 10] = [
    ShowcaseBuilding {
        id: "showcase-west-01",
        name: "West Gate Offices",
        x_min_m: -20.0,
        x_max_m: -7.2,
        z_min_m: -43.0,
        z_max_m: -29.0,
        height_m: 13.0,
    },
    ShowcaseBuilding {
        id: "showcase-west-02",
        name: "West Market",
        x_min_m: -18.5,
        x_max_m: -7.0,
        z_min_m: -26.0,
        z_max_m: -11.0,
        height_m: 18.0,
    },
    ShowcaseBuilding {
        id: "showcase-west-03",
        name: "West Civic Hall",
        x_min_m: -21.0,
        x_max_m: -7.3,
        z_min_m: -8.0,
        z_max_m: 7.0,
        height_m: 11.0,
    },
    ShowcaseBuilding {
        id: "showcase-west-04",
        name: "West Tower",
        x_min_m: -19.0,
        x_max_m: -7.1,
        z_min_m: 10.0,
        z_max_m: 25.0,
        height_m: 22.0,
    },
    ShowcaseBuilding {
        id: "showcase-west-05",
        name: "West Terrace",
        x_min_m: -20.5,
        x_max_m: -7.4,
        z_min_m: 28.0,
        z_max_m: 43.0,
        height_m: 15.0,
    },
    ShowcaseBuilding {
        id: "showcase-east-01",
        name: "East Station Annex",
        x_min_m: 7.2,
        x_max_m: 19.5,
        z_min_m: -43.0,
        z_max_m: -28.0,
        height_m: 17.0,
    },
    ShowcaseBuilding {
        id: "showcase-east-02",
        name: "East Arcade",
        x_min_m: 7.0,
        x_max_m: 21.0,
        z_min_m: -25.0,
        z_max_m: -10.0,
        height_m: 12.0,
    },
    ShowcaseBuilding {
        id: "showcase-east-03",
        name: "East Business Center",
        x_min_m: 7.4,
        x_max_m: 18.5,
        z_min_m: -7.0,
        z_max_m: 9.0,
        height_m: 20.0,
    },
    ShowcaseBuilding {
        id: "showcase-east-04",
        name: "East Library",
        x_min_m: 7.1,
        x_max_m: 20.0,
        z_min_m: 12.0,
        z_max_m: 27.0,
        height_m: 14.0,
    },
    ShowcaseBuilding {
        id: "showcase-east-05",
        name: "East Residence",
        x_min_m: 7.3,
        x_max_m: 19.0,
        z_min_m: 30.0,
        z_max_m: 44.0,
        height_m: 19.0,
    },
];

fn main() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let generated_dir = repo_root.join("target/plateau-sanjo-drive-demo");
    let source_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/sanjo_2025");
    let buildings = import_citygml_file(
        &source_dir.join("56383756_bldg_6697.gml"),
        &generated_dir.join("buildings"),
        &ImportOptions {
            tile_name: "sanjo-buildings".into(),
            coordinate_mode: CoordinateMode::GeographicDegrees,
            origin: Some(SANJO_ORIGIN),
            world_seed: 46,
            ..ImportOptions::default()
        },
    )
    .expect("import official PLATEAU building tile");
    let roads = import_citygml_file(
        &source_dir.join("56383756_tran_6697.gml"),
        &generated_dir.join("roads"),
        &ImportOptions {
            tile_name: "sanjo-roads".into(),
            coordinate_mode: CoordinateMode::GeographicDegrees,
            origin: Some(SANJO_ORIGIN),
            world_seed: 46,
            ..ImportOptions::default()
        },
    )
    .expect("import official PLATEAU road tile");
    let mut building_bundle =
        load_scene_bundle(&buildings.scene_path).expect("load generated PLATEAU buildings");
    let road_bundle = load_scene_bundle(&roads.scene_path).expect("load generated PLATEAU roads");
    flatten_buildings_to_road_datum(&mut building_bundle);
    let mut world = World::new();
    spawn_scene_bundle(
        &mut world,
        &building_bundle,
        None,
        SpawnSceneOptions::default(),
    )
    .expect("spawn generated PLATEAU buildings headlessly");
    spawn_scene_bundle(&mut world, &road_bundle, None, SpawnSceneOptions::default())
        .expect("spawn generated PLATEAU roads headlessly");
    assert_eq!(buildings.building_count, 213);
    assert_eq!(buildings.lod2_building_count, 1);
    assert_eq!(buildings.textured_surface_count, 37);
    assert_eq!(roads.road_count, 59);
    assert_eq!(roads.lane_count, 84);
    let imported_traffic =
        load_traffic_asset(&roads.traffic_path).expect("load imported PLATEAU traffic asset");
    let topology = build_traffic_topology(
        TrafficId::new("plateau:sanjo/topology").expect("Sanjo topology ID"),
        std::slice::from_ref(&imported_traffic.network),
        TopologyBuildConfig::default(),
    )
    .expect("build official Sanjo traffic topology");
    let city_scenario = build_city_traffic_scenario(&topology.network);
    let city_lane_route = &city_scenario.lane_routes[0];
    let city_runtime_route = city_scenario
        .routes
        .get(
            &TrafficId::new("plateau:sanjo/runtime-route-00")
                .expect("representative Sanjo route ID"),
        )
        .expect("representative Sanjo runtime route");
    let (city_left_turns, city_right_turns) =
        lane_route_turn_counts(&topology.network, city_lane_route);
    assert!(
        city_lane_route.connection_ids.len() >= 2,
        "Sanjo route must cross multiple junctions"
    );
    let signal_distances_m = city_scenario
        .signals
        .iter()
        .filter(|signal| signal.route_id == *city_runtime_route.id())
        .map(|signal| signal.stop_distance_m)
        .collect::<Vec<_>>();
    let replay_started = Instant::now();
    let forward_replay = replay_city_fleet(&topology.network, &city_scenario, false);
    let replay_elapsed = replay_started.elapsed();
    let reverse_replay = replay_city_fleet(&topology.network, &city_scenario, true);
    assert_eq!(forward_replay.stable_hash, reverse_replay.stable_hash);
    assert_eq!(forward_replay.signal_violation_count, 0);
    assert_eq!(forward_replay.collision_count, 0);
    assert!(
        forward_replay.minimum_gap_m >= 2.0 - 1.0e-9,
        "minimum Sanjo bumper gap was {:.12} m",
        forward_replay.minimum_gap_m
    );
    assert!(forward_replay.maximum_active_reservations > 0);
    let replay_hz = CITY_REPLAY_STEPS as f64 / replay_elapsed.as_secs_f64();
    assert!(
        replay_hz >= 60.0,
        "Sanjo 100-vehicle replay throughput {replay_hz:.1} Hz is below 60 Hz"
    );
    let showcase_lanes = select_station_road_lanes(&roads.lanes);
    let turn_lane = select_station_turn_lane(&roads.lanes, &showcase_lanes[0]);
    let turn_route = build_turn_route(&showcase_lanes[0], &turn_lane);
    println!(
        "official PLATEAU tile ready: buildings={} lod2={} textured_surfaces={} roads={} lanes={} junctions={} connections={} conflicts={} routes={} route_lanes={} route_m={:.1} left_turns={} right_turns={} actors={} signals={} reservations={} violations={} collisions={} minimum_gap_m={:.3} average_speed_m_s={:.2} waiting={} max_queue={} completed={} waiting_time_s={:.2} stable_hash={} replay_hz={:.1} triangles={}",
        buildings.building_count,
        buildings.lod2_building_count,
        buildings.textured_surface_count,
        roads.road_count,
        roads.lane_count,
        topology.stats.junction_count,
        topology.stats.connection_count,
        topology.stats.conflict_pair_count,
        city_scenario.routes.len(),
        city_lane_route.lane_ids.len(),
        city_runtime_route.total_length_m(),
        city_left_turns,
        city_right_turns,
        CITY_ACTOR_COUNT,
        city_scenario.signals.len(),
        forward_replay.maximum_active_reservations,
        forward_replay.signal_violation_count,
        forward_replay.collision_count,
        forward_replay.minimum_gap_m,
        forward_replay.flow.average_speed_m_s,
        forward_replay.flow.waiting_actor_count,
        forward_replay.maximum_queue_length,
        forward_replay.flow.completed_trip_count,
        forward_replay.flow.cumulative_waiting_time_s,
        forward_replay.stable_hash,
        replay_hz,
        buildings.triangle_count + roads.triangle_count,
    );

    let media_dir = std::env::var_os("RNE_MEDIA_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("docs/media"));
    let render_frame_count = std::env::var("RNE_RENDER_FRAME_COUNT")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(CAR_FRAME_COUNT)
        .clamp(1, CAR_FRAME_COUNT);
    let capture_frame_count = CAR_FRAME_COUNT;
    let frames_dir = generated_dir.join("lidar-frames");

    let mut city_scene = render_scene_from_world(&mut world);
    let mut mesh_roots = mesh_package_roots(&building_bundle);
    mesh_roots.extend(mesh_package_roots(&road_bundle));
    let root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    city_scene
        .resolve_mesh_assets_with_roots(&root_refs)
        .expect("resolve generated PLATEAU meshes");
    let building_footprints = building_footprints(&building_bundle);
    let street_fixtures =
        append_city_streetscape(&mut city_scene, &showcase_lanes, &building_footprints);
    append_intersection_markings(&mut city_scene, &turn_route);
    append_lane_markings(&mut city_scene, &showcase_lanes);
    append_runtime_route_pavement(&mut city_scene, &city_scenario.routes);
    let debug_overlay = TrafficDebugOverlay::from_environment();
    append_traffic_debug_overlay(
        &mut city_scene,
        &topology.network,
        &city_scenario.routes,
        city_runtime_route,
        &signal_distances_m,
        debug_overlay,
    );
    println!("traffic debug overlay: {debug_overlay:?}");
    let vehicle_assets = VehicleRenderAssets::load();
    let city_traffic =
        simulate_city_fleet_frames(&topology.network, &city_scenario, capture_frame_count);

    let scene_item_limit = if debug_overlay == TrafficDebugOverlay::default() {
        MAX_STATIC_SCENE_ITEMS
    } else {
        MAX_DEBUG_SCENE_ITEMS
    };
    assert!(
        city_scene.items.len() <= scene_item_limit,
        "PLATEAU scene leaves insufficient room for moving actors"
    );
    println!(
        "streetscape ready: fixtures={} static_scene_items={}",
        street_fixtures.len(),
        city_scene.items.len()
    );
    let mut camera = Camera::new(WIDTH, HEIGHT, 0.86);
    camera.far_m = 280.0;
    let primary_actor_index = (0..CITY_ACTOR_COUNT)
        .max_by(|left, right| {
            city_vehicle_motion_m(&city_traffic, *left)
                .total_cmp(&city_vehicle_motion_m(&city_traffic, *right))
                .then_with(|| right.cmp(left))
        })
        .expect("Sanjo fleet contains a camera target");
    println!(
        "camera follows actor={} motion_m={:.1}",
        primary_actor_index + 1,
        city_vehicle_motion_m(&city_traffic, primary_actor_index),
    );
    // The tracked actor's kinematic trajectory becomes a ghost that a dynamic-bicycle
    // vehicle chases; the sensors ride the dynamic chassis, the other 99 actors and
    // every traffic KPI stay on the untouched traffic contract.
    let dynamic_primary = apply_dynamic_primary(&city_traffic, primary_actor_index);
    println!(
        "dynamic primary ready: max_ghost_deviation_m={:.2} saturated_steps={}",
        dynamic_primary.maximum_deviation_m, dynamic_primary.saturated_steps,
    );
    let city_traffic = dynamic_primary.frames;
    let lidar_started = Instant::now();
    let lidar_capture =
        capture_city_lidar_frames(&mut world, &city_traffic, primary_actor_index, false);
    let lidar_elapsed = lidar_started.elapsed();
    let lidar_hz = lidar_capture.frames.len() as f64 / lidar_elapsed.as_secs_f64();
    assert_eq!(lidar_capture.frames.len(), capture_frame_count);
    assert!(
        lidar_capture.total_returns >= capture_frame_count * 24,
        "Sanjo LiDAR produced too few returns: {}",
        lidar_capture.total_returns
    );
    assert!(
        lidar_capture.multi_returns > 0,
        "Sanjo glass material must produce at least one later return"
    );
    assert!(
        lidar_hz >= RENDER_HZ as f64,
        "Sanjo physics-aware LiDAR throughput {lidar_hz:.1} Hz is below the {RENDER_HZ} Hz capture rate"
    );
    assert!(
        lidar_capture.saturated_returns > 0,
        "retroreflective licence plates must saturate the detector at least once"
    );
    println!(
        "physics-aware LiDAR ready: columns={} channels={} rays_per_scan={} frames={} returns={} multi_returns={} saturated={} average_intensity={:.3} scan_duration_s={:.4} stable_hash={} capture_hz={:.1}",
        LIDAR_RAY_COUNT,
        LIDAR_CHANNEL_COUNT,
        city_lidar_spec().rays_per_scan(),
        lidar_capture.frames.len(),
        lidar_capture.total_returns,
        lidar_capture.multi_returns,
        lidar_capture.saturated_returns,
        lidar_capture.average_intensity,
        lidar_capture
            .frames
            .first()
            .map(|frame| frame.cloud.scan_duration_s())
            .unwrap_or_default(),
        lidar_capture.stable_hash,
        lidar_hz,
    );
    let camera_started = Instant::now();
    let camera_capture = capture_city_camera_frames(
        &city_scene,
        &vehicle_assets,
        &city_traffic,
        primary_actor_index,
    );
    let camera_elapsed = camera_started.elapsed();
    assert_eq!(camera_capture.frames.len(), capture_frame_count);
    assert_eq!(
        camera_capture.pixels_per_frame,
        (CAMERA_WIDTH * CAMERA_HEIGHT) as usize
    );
    assert!(
        camera_capture.nearest_depth_m > 0.0 && camera_capture.nearest_depth_m.is_finite(),
        "onboard camera must observe scene geometry, got {}",
        camera_capture.nearest_depth_m
    );
    println!(
        "onboard camera ready: {}x{} fov_y_rad={:.3} frames={} nearest_depth_m={:.2} mean_center_depth_m={:.2} stable_hash={} capture_hz={:.1}",
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        CAMERA_FOV_Y_RAD,
        camera_capture.frames.len(),
        camera_capture.nearest_depth_m,
        camera_capture.mean_center_depth_m,
        camera_capture.stable_hash,
        camera_capture.frames.len() as f64 / camera_elapsed.as_secs_f64(),
    );
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless PLATEAU LiDAR and camera capture completed");
        return;
    }
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable after successful headless LiDAR capture: {error}");
            return;
        }
    };
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old PLATEAU LiDAR frames");
    }
    fs::create_dir_all(&frames_dir).expect("create PLATEAU LiDAR frames");
    fs::create_dir_all(&media_dir).expect("create media directory");
    let onboard_camera = Camera::new(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV_Y_RAD);
    let camera_spec = city_camera_spec();
    let mut previous_camera_pose: Option<Transform3> = None;
    for (frame, vehicles) in city_traffic.iter().take(render_frame_count).enumerate() {
        let primary = vehicles[primary_actor_index];
        let mut scene = city_scene.clone();
        append_city_runtime_signals(
            &mut scene,
            city_runtime_route,
            &signal_distances_m,
            frame * SIM_STEPS_PER_FRAME,
        );
        append_city_fleet(
            &mut scene,
            &vehicle_assets,
            vehicles,
            primary_actor_index,
            debug_overlay == TrafficDebugOverlay::default(),
        );
        // The onboard camera sees the world, not the LiDAR debug overlay, so it is
        // sampled from the scene before the point cloud markers are appended. Routing it
        // through the sensor model means the inset shows the real lens distortion,
        // rolling shutter, exposure, noise and vignetting, not a clean render.
        let onboard = sample_camera_rgbd_swept(
            &mut backend,
            &city_camera_sweep(previous_camera_pose, primary),
            &camera_spec,
            SimTime::from_ticks(frame as u64),
            &scene,
            city_camera_noise_key(frame),
        );
        previous_camera_pose = Some(vehicle_camera_transform(primary));

        append_lidar_intensity_overlay(&mut scene, &lidar_capture.frames[frame]);
        let car_camera = follow_camera(primary);
        let output = backend
            .render_scene_camera(&camera, &car_camera.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render PLATEAU car frame");
        let mut presented = cinematic_postprocess(
            &output.color.rgba8,
            &output.depth.depth_m,
            output.color.width,
            output.color.height,
            camera.far_m as f32,
        );
        blit_camera_insets(
            &mut FrameBuffer {
                pixels: &mut presented,
                width: output.color.width,
                height: output.color.height,
            },
            InsetImage {
                pixels: &onboard.rgb.rgba8,
                width: onboard.rgb.width,
                height: onboard.rgb.height,
            },
            &onboard.depth.depth_m,
            onboard_camera.far_m as f32,
        );
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &presented,
            output.color.width,
            output.color.height,
        )
        .expect("write PLATEAU car frame");
    }
    let lidar_gif_path = media_dir.join("plateau-lidar.gif");
    build_gif(&frames_dir, &lidar_gif_path).expect("encode PLATEAU LiDAR GIF");
    let poster_frame = render_frame_count.saturating_sub(1).min(110);
    image::open(frames_dir.join(format!("frame-{poster_frame:03}.png")))
        .expect("read PLATEAU LiDAR poster frame")
        .save(media_dir.join("plateau-lidar.png"))
        .expect("write PLATEAU LiDAR poster");
    fs::remove_dir_all(&frames_dir).expect("remove PLATEAU LiDAR frame directory");
    println!(
        "rendered official PLATEAU LiDAR media to {}",
        lidar_gif_path.display()
    );
}

#[cfg(test)]
fn select_city_lane_route(network: &TrafficNetwork) -> LaneRoute {
    select_city_lane_routes(network, 1)
        .into_iter()
        .next()
        .expect("official Sanjo topology must contain a directed multi-junction route")
}

fn city_vehicle_motion_m(frames: &[Vec<VehicleFrame>], actor_index: usize) -> f64 {
    frames
        .windows(2)
        .map(|pair| {
            (pair[1][actor_index].transform.translation
                - pair[0][actor_index].transform.translation)
                .length()
        })
        .sum()
}

fn select_city_lane_routes(network: &TrafficNetwork, route_count: usize) -> Vec<LaneRoute> {
    let mut lane_ids: Vec<_> = network
        .lanes
        .iter()
        .filter(|lane| {
            lane.allowed_actors
                .contains(&TrafficActorKind::MotorVehicle)
        })
        .map(|lane| lane.id.clone())
        .collect();
    lane_ids.sort();
    let connections = network
        .connections
        .iter()
        .map(|connection| (connection.id.clone(), connection))
        .collect::<BTreeMap<_, _>>();
    let mut unique_lane_sequences = BTreeSet::new();
    let mut candidates = Vec::new();
    for start_lane_id in &lane_ids {
        for goal_lane_id in &lane_ids {
            if start_lane_id == goal_lane_id {
                continue;
            }
            let Ok(route) = shortest_lane_route(
                network,
                start_lane_id,
                goal_lane_id,
                TrafficActorKind::MotorVehicle,
            ) else {
                continue;
            };
            if route.connection_ids.len() < 2
                || route.distance_m < 250.0
                || !unique_lane_sequences.insert(route.lane_ids.clone())
                || route_repeats_conflict_group(&route, &connections)
            {
                continue;
            }
            let (left_turns, right_turns) = lane_route_turn_counts(network, &route);
            let has_both_turns = left_turns > 0 && right_turns > 0;
            candidates.push((has_both_turns, route));
        }
    }
    candidates.sort_by(|(left_both, left), (right_both, right)| {
        right_both
            .cmp(left_both)
            .then_with(|| right.connection_ids.len().cmp(&left.connection_ids.len()))
            .then_with(|| left.distance_m.total_cmp(&right.distance_m))
            .then_with(|| left.lane_ids.cmp(&right.lane_ids))
    });
    let mut selected = Vec::with_capacity(route_count);
    if let Some((_, route)) = candidates.first() {
        selected.push(route.clone());
    }
    while selected.len() < route_count && selected.len() < candidates.len() {
        let used_connections = selected
            .iter()
            .flat_map(|route| route.connection_ids.iter().cloned())
            .collect::<BTreeSet<_>>();
        let used_starts = selected
            .iter()
            .filter_map(|route| route.lane_ids.first().cloned())
            .collect::<BTreeSet<_>>();
        let used_goals = selected
            .iter()
            .filter_map(|route| route.lane_ids.last().cloned())
            .collect::<BTreeSet<_>>();
        let next = candidates
            .iter()
            .filter(|(_, candidate)| {
                !selected
                    .iter()
                    .any(|route| route.lane_ids == candidate.lane_ids)
            })
            .max_by(|(_, left), (_, right)| {
                city_route_diversity_score(
                    left,
                    &connections,
                    &used_connections,
                    &used_starts,
                    &used_goals,
                )
                .cmp(&city_route_diversity_score(
                    right,
                    &connections,
                    &used_connections,
                    &used_starts,
                    &used_goals,
                ))
                .then_with(|| right.lane_ids.cmp(&left.lane_ids))
            })
            .map(|(_, route)| route.clone());
        let Some(next) = next else {
            break;
        };
        selected.push(next);
    }
    assert_eq!(
        selected.len(),
        route_count,
        "official Sanjo topology needs {route_count} diverse routes"
    );
    selected
}

fn route_repeats_conflict_group(
    route: &LaneRoute,
    connections: &BTreeMap<TrafficId, &rne_traffic::TrafficConnection>,
) -> bool {
    let mut groups = BTreeSet::new();
    route.connection_ids.iter().any(|connection_id| {
        let connection = connections
            .get(connection_id)
            .expect("planned connection must exist");
        !connection.conflict_connection_ids.is_empty()
            && connection
                .junction_id
                .as_ref()
                .is_some_and(|junction_id| !groups.insert(junction_id.clone()))
    })
}

fn city_route_diversity_score(
    route: &LaneRoute,
    connections: &BTreeMap<TrafficId, &rne_traffic::TrafficConnection>,
    used_connections: &BTreeSet<TrafficId>,
    used_starts: &BTreeSet<TrafficId>,
    used_goals: &BTreeSet<TrafficId>,
) -> (bool, usize, usize, usize, usize) {
    let conflict_count = route
        .connection_ids
        .iter()
        .filter(|connection_id| {
            connections.get(*connection_id).is_some_and(|connection| {
                connection
                    .conflict_connection_ids
                    .iter()
                    .any(|conflict_id| used_connections.contains(conflict_id))
            })
        })
        .count();
    let new_connection_count = route
        .connection_ids
        .iter()
        .filter(|connection_id| !used_connections.contains(*connection_id))
        .count();
    let endpoint_novelty = usize::from(
        route
            .lane_ids
            .first()
            .is_some_and(|lane_id| !used_starts.contains(lane_id)),
    ) + usize::from(
        route
            .lane_ids
            .last()
            .is_some_and(|lane_id| !used_goals.contains(lane_id)),
    );
    (
        conflict_count > 0,
        conflict_count,
        new_connection_count,
        endpoint_novelty,
        route.connection_ids.len(),
    )
}

fn build_city_traffic_scenario(network: &TrafficNetwork) -> CityTrafficScenario {
    const ACTOR_STREAM_DOMAIN: u64 = 0x5341_4E4A_4F5F_4341;
    let lane_routes = select_city_lane_routes(network, CITY_ROUTE_COUNT);
    let mut routes = TrafficRouteCatalog::default();
    for (index, lane_route) in lane_routes.iter().enumerate() {
        routes
            .insert(
                materialize_lane_route(
                    network,
                    lane_route,
                    TrafficId::new(format!("plateau:sanjo/runtime-route-{index:02}"))
                        .expect("Sanjo runtime route ID"),
                    false,
                )
                .expect("materialize official Sanjo runtime route"),
            )
            .expect("insert distinct Sanjo runtime route");
    }

    let world_random = WorldRandom::new(46);
    let mut route_actor_indices = vec![Vec::new(); routes.len()];
    for actor_index in 0..CITY_ACTOR_COUNT {
        route_actor_indices[actor_index % routes.len()].push(actor_index);
    }
    let runtime_routes = routes.iter().map(|(_, route)| route).collect::<Vec<_>>();
    let mut actors: Vec<Option<CityActorSpec>> = vec![None; CITY_ACTOR_COUNT];
    for (route_index, actor_indices) in route_actor_indices.iter().enumerate() {
        let route = runtime_routes[route_index];
        let mut distance_m = 6.0;
        let mut previous_length_m = 0.0;
        for actor_index in actor_indices {
            let mut rng = world_random.stream(RandomStreamId::new(
                ACTOR_STREAM_DOMAIN ^ *actor_index as u64,
            ));
            let class = match rng.uniform_usize(12) {
                0 => CityVehicleClass::Bus,
                1..=3 => CityVehicleClass::Van,
                4..=7 => CityVehicleClass::Compact,
                _ => CityVehicleClass::Sedan,
            };
            let length_m = class.length_m();
            let seeded_gap_m = rng.uniform_f64(4.0, 7.0);
            if previous_length_m > 0.0 {
                distance_m += (previous_length_m + length_m) * 0.5 + seeded_gap_m;
            }
            distance_m = city_lane_spawn_distance_m(route, distance_m, length_m);
            while actors.iter().flatten().any(|other| {
                let other_route = routes
                    .get(&other.route_id)
                    .expect("assigned Sanjo actor route");
                city_vehicle_footprints_overlap(
                    route,
                    distance_m,
                    length_m,
                    other_route,
                    other.distance_m,
                    other.class.length_m(),
                )
            }) {
                distance_m += 1.0;
                distance_m = city_lane_spawn_distance_m(route, distance_m, length_m);
                assert!(
                    distance_m < route.total_length_m() * 0.65,
                    "Sanjo route {route_index} cannot place its fleet without overlap"
                );
            }
            let class_speed_m_s = match class {
                CityVehicleClass::Compact => 9.2,
                CityVehicleClass::Sedan => 9.0,
                CityVehicleClass::Van => 8.3,
                CityVehicleClass::Bus => 7.2,
            };
            assert!(
                distance_m < route.total_length_m() * 0.65,
                "Sanjo route {route_index} is too short for its deterministic fleet"
            );
            actors[*actor_index] = Some(CityActorSpec {
                uuid: Uuid::from_u128(*actor_index as u128 + 1),
                route_id: route.id().clone(),
                distance_m,
                desired_speed_m_s: class_speed_m_s + rng.uniform_f64(-0.5, 0.8),
                departure_time_s: if *actor_index == 0 {
                    0.0
                } else {
                    rng.uniform_f64(0.0, 5.0)
                },
                class,
            });
            previous_length_m = length_m;
        }
    }

    let mut signals = Vec::new();
    for (route_index, (lane_route, runtime_route)) in
        lane_routes.iter().zip(runtime_routes).enumerate()
    {
        for (signal_index, stop_distance_m) in
            city_signal_distances_m(network, lane_route, runtime_route)
                .into_iter()
                .enumerate()
        {
            let global_index = route_index * CITY_SIGNAL_COUNT + signal_index;
            signals.push(CitySignalSpec {
                id: TrafficId::new(format!(
                    "plateau:sanjo/runtime-signal-{route_index:02}-{signal_index:02}"
                ))
                .expect("Sanjo signal control ID"),
                route_id: runtime_route.id().clone(),
                stop_distance_m,
                phase_offset_steps: global_index as u64 * 120,
            });
        }
    }
    CityTrafficScenario {
        lane_routes,
        routes,
        actors: actors
            .into_iter()
            .map(|actor| actor.expect("every Sanjo actor is assigned"))
            .collect(),
        signals,
    }
}

fn city_lane_spawn_distance_m(
    route: &TrafficRoute,
    mut distance_m: f64,
    vehicle_length_m: f64,
) -> f64 {
    loop {
        let blocked = route.movements().iter().find(|movement| {
            distance_m >= movement.entry_distance_m - vehicle_length_m * 0.5 - 2.0
                && distance_m < movement.exit_distance_m + vehicle_length_m * 0.5 + 2.0
        });
        let Some(blocked) = blocked else {
            return distance_m;
        };
        distance_m = blocked.exit_distance_m + vehicle_length_m * 0.5 + 2.0;
    }
}

fn city_vehicle_footprints_overlap(
    left_route: &TrafficRoute,
    left_distance_m: f64,
    left_length_m: f64,
    right_route: &TrafficRoute,
    right_distance_m: f64,
    right_length_m: f64,
) -> bool {
    let left = left_route.sample(left_distance_m);
    let right = right_route.sample(right_distance_m);
    let left_forward = [left.yaw_rad.cos(), -left.yaw_rad.sin()];
    let left_right = [-left_forward[1], left_forward[0]];
    let right_forward = [right.yaw_rad.cos(), -right.yaw_rad.sin()];
    let right_right = [-right_forward[1], right_forward[0]];
    let delta = [
        right.position_m[0] - left.position_m[0],
        right.position_m[2] - left.position_m[2],
    ];
    [left_forward, left_right, right_forward, right_right]
        .into_iter()
        .all(|axis| {
            let center_distance_m = city_dot2(delta, axis).abs();
            let left_radius_m = left_length_m * 0.5 * city_dot2(left_forward, axis).abs()
                + city_dot2(left_right, axis).abs();
            let right_radius_m = right_length_m * 0.5 * city_dot2(right_forward, axis).abs()
                + city_dot2(right_right, axis).abs();
            center_distance_m < left_radius_m + right_radius_m - 1.0e-9
        })
}

fn city_dot2(left: [f64; 2], right: [f64; 2]) -> f64 {
    left[0] * right[0] + left[1] * right[1]
}

fn lane_route_turn_counts(network: &TrafficNetwork, route: &LaneRoute) -> (usize, usize) {
    let mut left_turns = 0;
    let mut right_turns = 0;
    for connection_id in &route.connection_ids {
        let movement = network
            .connections
            .iter()
            .find(|connection| &connection.id == connection_id)
            .expect("planned connection must exist")
            .movement;
        match movement {
            MovementKind::Left => left_turns += 1,
            MovementKind::Right => right_turns += 1,
            _ => {}
        }
    }
    (left_turns, right_turns)
}

fn city_signal_distances_m(
    network: &TrafficNetwork,
    lane_route: &LaneRoute,
    runtime_route: &TrafficRoute,
) -> Vec<f64> {
    (1..=CITY_SIGNAL_COUNT)
        .map(|signal_index| {
            let route_index =
                signal_index * lane_route.connection_ids.len() / (CITY_SIGNAL_COUNT + 1);
            let connection_id = &lane_route.connection_ids[route_index];
            let connection = network
                .connections
                .iter()
                .find(|connection| &connection.id == connection_id)
                .expect("planned signal connection");
            nearest_route_distance_m(runtime_route, Vec3::from_array(connection.path_m[0]))
        })
        .collect()
}

fn city_signal_aspect(step: u64, signal_index: usize) -> SignalAspect {
    let phase_step = (step + signal_index as u64 * 120) % 360;
    if phase_step < 180 {
        SignalAspect::Red
    } else {
        SignalAspect::Green
    }
}

fn city_signal_aspect_with_offset(step: u64, phase_offset_steps: u64) -> SignalAspect {
    let phase_step = (step + phase_offset_steps) % 360;
    if phase_step < 180 {
        SignalAspect::Red
    } else {
        SignalAspect::Green
    }
}

fn city_signal_controls(scenario: &CityTrafficScenario) -> TrafficSignalControls {
    let mut controls = TrafficSignalControls::default();
    for signal in &scenario.signals {
        controls
            .insert(TrafficSignalControl {
                id: signal.id.clone(),
                route_id: signal.route_id.clone(),
                stop_distance_m: signal.stop_distance_m,
                aspect: city_signal_aspect_with_offset(0, signal.phase_offset_steps),
            })
            .expect("insert Sanjo signal control");
    }
    controls
}

fn spawn_city_fleet(
    world: &mut World,
    scenario: &CityTrafficScenario,
    reverse_spawn_order: bool,
) -> Vec<Entity> {
    let actor_indices: Vec<_> = if reverse_spawn_order {
        (0..scenario.actors.len()).rev().collect()
    } else {
        (0..scenario.actors.len()).collect()
    };
    let mut actor_entities = vec![None; scenario.actors.len()];
    for index in actor_indices {
        let actor = &scenario.actors[index];
        let route = scenario
            .routes
            .get(&actor.route_id)
            .expect("scenario actor route");
        let distance_m = actor.distance_m;
        let sample = route.sample(distance_m);
        actor_entities[index] = Some(
            world
                .spawn((
                    TrafficActor::motor_vehicle(),
                    EntityUuid(actor.uuid),
                    TrafficRouteFollower {
                        route_id: actor.route_id.clone(),
                        distance_m,
                        speed_m_s: 0.0,
                        desired_speed_m_s: actor.desired_speed_m_s,
                        length_m: actor.class.length_m(),
                    },
                    TrafficDeparture {
                        departure_time_s: actor.departure_time_s,
                    },
                    TrafficPose {
                        position_m: sample.position_m,
                        yaw_rad: sample.yaw_rad,
                    },
                ))
                .id(),
        );
    }
    actor_entities
        .into_iter()
        .map(|entity| entity.expect("every Sanjo actor is spawned"))
        .collect()
}

fn replay_city_fleet(
    network: &TrafficNetwork,
    scenario: &CityTrafficScenario,
    reverse_spawn_order: bool,
) -> CityReplayResult {
    let mut controls = city_signal_controls(scenario);
    let mut conflict_controls =
        TrafficConflictControls::from_network_routes(network, &scenario.routes, 24.0)
            .expect("build Sanjo junction conflict controls");
    assert!(
        !conflict_controls.is_empty(),
        "diverse Sanjo routes must exercise conflict reservations"
    );
    let mut world = World::new();
    spawn_city_fleet(&mut world, scenario, reverse_spawn_order);
    let delta = SimDuration::from_hertz(Hertz::new(SIM_HZ as f64));
    let mut clock = SimClock::new(delta);
    let mut runtime = TrafficRuntime::default();
    let mut collision_count = 0;
    let mut signal_violation_count = 0;
    let mut minimum_gap_m = f64::INFINITY;
    let mut stable_hash = 0;
    let mut maximum_active_reservations = 0;
    let mut maximum_queue_length = 0;
    let mut flow = TrafficFlowMetrics::default();
    for step in 1..=CITY_REPLAY_STEPS {
        for signal in &scenario.signals {
            controls
                .set_aspect(
                    &signal.id,
                    city_signal_aspect_with_offset(step, signal.phase_offset_steps),
                )
                .expect("update Sanjo signal phase");
        }
        assert_eq!(clock.advance(delta), 1);
        let report = advance_reserved_kinematic_traffic(
            &mut world,
            &scenario.routes,
            KinematicTrafficControls::new(&controls, &mut conflict_controls),
            &mut runtime,
            clock.sim_time(),
            delta,
            KinematicTrafficConfig::default(),
        )
        .expect("advance Sanjo traffic");
        collision_count += report.collision_count;
        signal_violation_count += report.signal_violation_count;
        if let Some(gap_m) = report.minimum_observed_gap_m {
            minimum_gap_m = minimum_gap_m.min(gap_m);
        }
        stable_hash = report.stable_state_hash;
        maximum_active_reservations =
            maximum_active_reservations.max(report.active_reservation_count);
        maximum_queue_length = maximum_queue_length.max(report.flow.maximum_queue_length);
        flow = report.flow;
    }
    CityReplayResult {
        stable_hash,
        collision_count,
        signal_violation_count,
        minimum_gap_m,
        maximum_active_reservations,
        maximum_queue_length,
        flow,
    }
}

fn simulate_city_fleet_frames(
    network: &TrafficNetwork,
    scenario: &CityTrafficScenario,
    frame_count: usize,
) -> Vec<Vec<VehicleFrame>> {
    let mut controls = city_signal_controls(scenario);
    let mut conflict_controls =
        TrafficConflictControls::from_network_routes(network, &scenario.routes, 24.0)
            .expect("build Sanjo render conflict controls");
    let mut world = World::new();
    let entities = spawn_city_fleet(&mut world, scenario, false);
    let delta = SimDuration::from_hertz(Hertz::new(SIM_HZ as f64));
    let mut clock = SimClock::new(delta);
    let mut runtime = TrafficRuntime::default();
    let mut wheel_rotation_rad = vec![0.0; CITY_ACTOR_COUNT];
    let mut previous_speed_m_s = vec![0.0; CITY_ACTOR_COUNT];
    let mut frames = Vec::with_capacity(frame_count);
    for frame_index in 0..frame_count {
        frames.push(
            entities
                .iter()
                .enumerate()
                .map(|(index, entity)| {
                    let pose = world.get::<TrafficPose>(*entity).expect("render pose");
                    let follower = world
                        .get::<TrafficRouteFollower>(*entity)
                        .expect("render follower");
                    let route = scenario
                        .routes
                        .get(&follower.route_id)
                        .expect("render follower route");
                    VehicleFrame {
                        transform: Transform3::from_translation_rotation(
                            Vec3::from_array(pose.position_m) + Vec3::new(0.0, 0.60, 0.0),
                            Quat::from_rotation_y(pose.yaw_rad),
                        ),
                        speed_m_s: follower.speed_m_s,
                        steering_rad: route_steering_rad(route, follower.distance_m, pose.yaw_rad),
                        wheel_rotation_rad: wheel_rotation_rad[index],
                        braking: follower.speed_m_s + 0.05 < previous_speed_m_s[index],
                        class: scenario.actors[index].class,
                    }
                })
                .collect(),
        );
        for (index, entity) in entities.iter().enumerate() {
            previous_speed_m_s[index] = world
                .get::<TrafficRouteFollower>(*entity)
                .expect("captured render follower")
                .speed_m_s;
        }
        for substep in 0..SIM_STEPS_PER_FRAME {
            let step = (frame_index * SIM_STEPS_PER_FRAME + substep + 1) as u64;
            for signal in &scenario.signals {
                controls
                    .set_aspect(
                        &signal.id,
                        city_signal_aspect_with_offset(step, signal.phase_offset_steps),
                    )
                    .expect("update Sanjo render signal");
            }
            assert_eq!(clock.advance(delta), 1);
            let report = advance_reserved_kinematic_traffic(
                &mut world,
                &scenario.routes,
                KinematicTrafficControls::new(&controls, &mut conflict_controls),
                &mut runtime,
                clock.sim_time(),
                delta,
                KinematicTrafficConfig::default(),
            )
            .expect("advance Sanjo render traffic");
            assert_eq!(report.collision_count, 0);
            assert_eq!(report.signal_violation_count, 0);
            for (index, entity) in entities.iter().enumerate() {
                let speed_m_s = world
                    .get::<TrafficRouteFollower>(*entity)
                    .expect("advanced render follower")
                    .speed_m_s;
                wheel_rotation_rad[index] += speed_m_s * delta.as_seconds().value() / 0.36;
            }
        }
    }
    frames
}

/// Result of replacing the tracked actor's motion with the dynamic bicycle model.
#[derive(Clone, Debug, PartialEq)]
struct DynamicPrimaryResult {
    /// Traffic frames with the primary actor's row rewritten per frame.
    frames: Vec<Vec<VehicleFrame>>,
    /// Largest distance between the dynamic vehicle and its kinematic ghost, meters.
    maximum_deviation_m: f64,
    /// Simulation steps during which either axle was friction saturated.
    saturated_steps: usize,
}

/// Time the dynamic primary looks ahead along its ghost's trajectory, seconds.
const DYNAMIC_PRIMARY_LOOKAHEAD_S: f64 = 0.6;
/// Upper bound on how far the dynamic primary may stray from its ghost, meters.
const DYNAMIC_PRIMARY_DEVIATION_LIMIT_M: f64 = 2.0;

/// Re-drives the tracked actor with the planar dynamic bicycle model.
///
/// The deterministic traffic contract — routes, signals, reservations, and the other
/// 99 actors — is untouched: `rne_traffic` stays free of physics backends, and the
/// swap happens here in the example layer. The tracked actor's kinematic trajectory
/// becomes a *ghost* that a [`VehicleDynamics`] vehicle chases with pure pursuit, so
/// its pose gains tire slip, understeer in tight turns, and steering-actuator sway.
/// The LiDAR and camera then ride a chassis that answers through forces, not the
/// ghost that ignores them.
fn apply_dynamic_primary(
    traffic_frames: &[Vec<VehicleFrame>],
    primary_actor_index: usize,
) -> DynamicPrimaryResult {
    let ghost: Vec<VehicleFrame> = traffic_frames
        .iter()
        .map(|vehicles| vehicles[primary_actor_index])
        .collect();
    assert!(!ghost.is_empty(), "traffic frames are empty");

    // Interpolated ghost pose at an arbitrary time, clamped to the capture window.
    let frame_dt_s = 1.0 / RENDER_HZ as f64;
    let ghost_position = |time_s: f64| -> Vec3 {
        let exact = (time_s / frame_dt_s).max(0.0);
        let index = (exact.floor() as usize).min(ghost.len() - 1);
        let next = (index + 1).min(ghost.len() - 1);
        let t = (exact - index as f64).clamp(0.0, 1.0);
        ghost[index]
            .transform
            .translation
            .lerp(ghost[next].transform.translation, t)
    };

    let mut world = World::new();
    let vehicle = spawn_named(&mut world, "dynamic_primary");
    world.entity_mut(vehicle).insert((
        AckermannDrive {
            wheelbase_m: VehicleDynamics::default().wheelbase_m(),
            max_speed_m_s: 20.0,
            max_steering_rad: 0.6,
            max_acceleration_m_s2: 3.0,
            max_deceleration_m_s2: 6.0,
            max_steering_rate_rad_s: 4.0,
            speed_m_s: ghost[0].speed_m_s,
            target_speed_m_s: ghost[0].speed_m_s,
            ..AckermannDrive::default()
        },
        VehicleDynamics {
            // A short steering-actuator lag keeps the chase visibly physical.
            steering_lag_s: 0.08,
            ..VehicleDynamics::default()
        },
        ghost[0].transform,
        RigidBody::default(),
    ));

    let dt = SimDuration::from_hertz(Hertz::new(SIM_HZ as f64));
    let dt_s = dt.as_seconds().value();
    let mut frames = traffic_frames.to_vec();
    let mut maximum_deviation_m = 0.0_f64;
    let mut saturated_steps = 0_usize;
    let mut wheel_rotation_rad = ghost[0].wheel_rotation_rad;

    for (frame_index, vehicles) in frames.iter_mut().enumerate() {
        for step in 0..SIM_STEPS_PER_FRAME {
            let time_s = frame_index as f64 * frame_dt_s + step as f64 * dt_s;
            let ghost_frame = &ghost[frame_index];
            let transform = *world.get::<Transform3>(vehicle).expect("primary transform");

            // Chase the ghost's future position; match the ghost's commanded speed so
            // signal stops and car-following gaps carry over from the traffic layer.
            let target = ghost_position(time_s + DYNAMIC_PRIMARY_LOOKAHEAD_S);
            let steering = pure_pursuit_steering(
                &transform,
                target,
                VehicleDynamics::default().wheelbase_m(),
                (target - transform.translation).length().max(1.0),
            );
            {
                let mut drive = world.get_mut::<AckermannDrive>(vehicle).expect("drive");
                drive.target_steering_rad =
                    steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
                drive.target_speed_m_s = ghost_frame.speed_m_s;
            }
            vehicle_dynamics(&mut world, dt);

            let dynamics = world
                .get::<VehicleDynamics>(vehicle)
                .expect("primary dynamics");
            if dynamics.front_saturated || dynamics.rear_saturated {
                saturated_steps += 1;
            }
        }

        let transform = *world.get::<Transform3>(vehicle).expect("primary transform");
        let drive = world.get::<AckermannDrive>(vehicle).expect("drive").clone();
        let ghost_frame = ghost[frame_index];
        maximum_deviation_m = maximum_deviation_m
            .max((transform.translation - ghost_frame.transform.translation).length());
        wheel_rotation_rad += drive.speed_m_s * frame_dt_s / 0.36;

        vehicles[primary_actor_index] = VehicleFrame {
            transform,
            speed_m_s: drive.speed_m_s,
            steering_rad: drive.steering_rad,
            wheel_rotation_rad,
            braking: ghost_frame.braking,
            class: ghost_frame.class,
        };
    }

    assert!(
        maximum_deviation_m < DYNAMIC_PRIMARY_DEVIATION_LIMIT_M,
        "dynamic primary strayed {maximum_deviation_m:.2} m from its ghost"
    );

    DynamicPrimaryResult {
        frames,
        maximum_deviation_m,
        saturated_steps,
    }
}

fn capture_city_lidar_frames(
    world: &mut World,
    traffic_frames: &[Vec<VehicleFrame>],
    primary_actor_index: usize,
    reverse_vehicle_spawn_order: bool,
) -> CityLidarCapture {
    assign_city_lidar_materials(world);

    // Ground plane under the whole tile. The imported road surfaces only cover the
    // carriageway, so without this the downward elevation channels return nothing
    // over grass and sidewalks and the signature concentric LiDAR rings appear as a
    // partial crescent. Real ground reflects everywhere; grass is diffuse and dim.
    let ground = spawn_named(world, "lidar_ground_plane");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(400.0, 0.05, 400.0),
            },
            ..Collider::default()
        },
        LidarMaterial::new(0.14, 0.0, 0.95),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.06, 0.0), Quat::IDENTITY),
    ));
    let actor_count = traffic_frames
        .first()
        .map(Vec::len)
        .expect("LiDAR capture requires traffic frames");
    let mut spawn_indices = (0..actor_count)
        .filter(|index| *index != primary_actor_index)
        .collect::<Vec<_>>();
    if reverse_vehicle_spawn_order {
        spawn_indices.reverse();
    }
    let mut collider_entities = vec![None; actor_count];
    let mut plate_entities = vec![None; actor_count];
    for actor_index in spawn_indices {
        let vehicle = traffic_frames[0][actor_index];
        let entity = spawn_named(world, format!("lidar_vehicle_{actor_index:03}"));
        world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(
                        vehicle.class.length_m() * 0.5,
                        vehicle.class.height_m() * 0.5,
                        vehicle.class.width_m() * 0.5,
                    ),
                },
                ..Collider::default()
            },
            LidarMaterial::painted_metal(),
            lidar_vehicle_transform(vehicle),
        ));
        collider_entities[actor_index] = Some(entity);

        // Retroreflective licence plates are what make vehicles saturate a real
        // detector, so they exercise the saturation and bloom path.
        let plate = spawn_named(world, format!("lidar_plate_{actor_index:03}"));
        world.entity_mut(plate).insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(0.02, 0.08, 0.17),
                },
                ..Collider::default()
            },
            LidarMaterial::licence_plate(),
            lidar_plate_transform(vehicle),
        ));
        plate_entities[actor_index] = Some(plate);
    }

    let mut physics = RapierBackend::new();
    let physics_world = physics
        .create_world(PhysicsWorldDesc::default())
        .expect("create Sanjo LiDAR physics world");
    let spec = city_lidar_spec();
    let world_random = WorldRandom::new(46);
    let mut frames = Vec::with_capacity(traffic_frames.len());
    let mut stable_hash = 0xcbf29ce484222325_u64;
    let mut total_returns = 0_usize;
    let mut multi_returns = 0_usize;
    let mut saturated_returns = 0_usize;
    let mut intensity_sum = 0.0_f64;
    let mut previous_mount: Option<Transform3> = None;

    for (frame_index, vehicles) in traffic_frames.iter().enumerate() {
        for (actor_index, entity) in collider_entities.iter().enumerate() {
            let Some(entity) = entity else {
                continue;
            };
            if let Some(mut transform) = world.get_mut::<Transform3>(*entity) {
                *transform = lidar_vehicle_transform(vehicles[actor_index]);
            }
        }
        for (actor_index, entity) in plate_entities.iter().enumerate() {
            let Some(entity) = entity else {
                continue;
            };
            if let Some(mut transform) = world.get_mut::<Transform3>(*entity) {
                *transform = lidar_plate_transform(vehicles[actor_index]);
            }
        }
        physics
            .sync_from_ecs(world, physics_world)
            .expect("sync Sanjo LiDAR colliders");
        let primary = vehicles[primary_actor_index];
        let mount = lidar_mount_transform(primary);
        // The scanner sweeps while the host vehicle drives, so each azimuth column is
        // cast from the pose interpolated between the previous and current frame.
        let sweep = LidarSweep::new(previous_mount.unwrap_or(mount), mount);
        let cloud = sample_lidar_swept(
            &physics,
            physics_world,
            world,
            &sweep,
            &spec,
            SensorNoiseKey::new(
                world_random.seed(),
                spec.seed,
                LIDAR_STREAM_ID,
                frame_index as u64 + 1,
            ),
        );
        assert!(cloud.attributes_are_aligned());
        total_returns += cloud.points_m.len();
        multi_returns += cloud
            .return_indices
            .iter()
            .filter(|return_index| **return_index > 1)
            .count();
        saturated_returns += cloud
            .intensities
            .iter()
            .filter(|intensity| f64::from(**intensity) >= LIDAR_SATURATION_INTENSITY - 1e-6)
            .count();
        intensity_sum += cloud
            .intensities
            .iter()
            .map(|intensity| f64::from(*intensity))
            .sum::<f64>();
        stable_hash = hash_lidar_cloud(stable_hash, &cloud);
        previous_mount = Some(mount);
        frames.push(CityLidarFrame { mount, cloud });
    }

    CityLidarCapture {
        frames,
        stable_hash,
        total_returns,
        multi_returns,
        saturated_returns,
        average_intensity: if total_returns == 0 {
            0.0
        } else {
            intensity_sum / total_returns as f64
        },
    }
}

fn city_lidar_spec() -> LidarSpec {
    LidarSpec {
        ray_count: LIDAR_RAY_COUNT,
        min_angle_rad: -std::f64::consts::PI,
        max_angle_rad: std::f64::consts::PI,
        channel_count: LIDAR_CHANNEL_COUNT,
        min_elevation_rad: LIDAR_MIN_ELEVATION_RAD,
        max_elevation_rad: LIDAR_MAX_ELEVATION_RAD,
        rotation_period_s: LIDAR_ROTATION_PERIOD_S,
        min_range_m: 0.8,
        max_range_m: LIDAR_MAX_RANGE_M,
        max_returns: 2,
        wavelength_nm: 905.0,
        beam_divergence_rad: 0.003,
        beam_sample_count: LIDAR_BEAM_SAMPLE_COUNT,
        mixed_pixel_threshold_m: 0.35,
        range_noise_stddev_m: 0.012,
        intensity_noise_stddev: 0.004,
        solar_noise_floor: 0.003,
        dropout_probability: 0.01,
        saturation_intensity: LIDAR_SATURATION_INTENSITY,
        bloom_gain: 0.06,
        bloom_column_radius: 2,
        backscatter_probability_scale: 0.0,
        minimum_intensity: 0.002,
        seed: 46_905,
        // The rendered scene is a clear sunny day, so the atmosphere matches it: a
        // trace of haze and dust, no precipitation, and no aerosol backscatter.
        // Rain and backscatter are exercised by the rne_sensor unit tests; enabling
        // them here would sprinkle floating returns through visibly clear air.
        atmosphere: LidarAtmosphere {
            fog_extinction_per_m: 0.000_2,
            rain_rate_mm_h: 0.0,
            dust_density_mg_m3: 0.2,
            snow_rate_mm_h: 0.0,
        },
        domain_randomization: LidarDomainRandomization {
            fog_extinction_range_per_m: [0.0, 0.000_4],
            rain_rate_range_mm_h: [0.0, 0.0],
            dust_density_range_mg_m3: [0.0, 0.4],
            snow_rate_range_mm_h: [0.0, 0.0],
        },
        ..LidarSpec::default()
    }
}

fn assign_city_lidar_materials(world: &mut World) {
    let assignments = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            world.get::<Collider>(entity)?;
            let name = world
                .get::<Name>(entity)
                .map(|name| name.0.as_str())
                .unwrap_or_default();
            let material = if name.contains("road") || name.contains("ground") {
                LidarMaterial::dry_asphalt()
            } else if name.contains("building") && stable_name_hash(name).is_multiple_of(3) {
                LidarMaterial::clear_glass()
            } else if name.contains("building") {
                LidarMaterial::concrete()
            } else {
                LidarMaterial::default()
            };
            Some((entity, material))
        })
        .collect::<Vec<_>>();
    for (entity, material) in assignments {
        world.entity_mut(entity).insert(material);
    }
}

fn lidar_vehicle_transform(vehicle: VehicleFrame) -> Transform3 {
    Transform3::from_translation_rotation(
        Vec3::new(
            vehicle.transform.translation.x,
            vehicle.class.height_m() * 0.5,
            vehicle.transform.translation.z,
        ),
        vehicle.transform.rotation,
    )
}

fn city_camera_spec() -> CameraSpec {
    CameraSpec {
        width: CAMERA_WIDTH,
        height: CAMERA_HEIGHT,
        fov_y_rad: CAMERA_FOV_Y_RAD,
        seed: CAMERA_STREAM_ID,
        // Mild barrel distortion typical of a wide automotive lens.
        distortion: CameraDistortion {
            k1: -0.26,
            k2: 0.07,
            ..CameraDistortion::default()
        },
        // A CMOS sensor reads the frame out in roughly 20 ms while the car drives.
        readout_time_s: CAMERA_READOUT_TIME_S,
        rolling_shutter_bands: CAMERA_ROLLING_SHUTTER_BANDS,
        auto_exposure_target_luminance: 0.42,
        auto_exposure_max_ev: 2.0,
        shot_noise_scale: 0.0006,
        read_noise_stddev: 0.003,
        vignette_strength: 0.35,
        ..CameraSpec::default()
    }
}

/// Returns the readout sweep for one frame of the onboard camera.
///
/// The sensor scans its rows out while the car drives, so the sweep runs from the pose
/// at the previous frame to the pose now. The first frame has no predecessor and is
/// captured as a global shutter.
fn city_camera_sweep(previous_pose: Option<Transform3>, primary: VehicleFrame) -> CameraSweep {
    let pose = vehicle_camera_transform(primary);
    CameraSweep::new(previous_pose.unwrap_or(pose), pose)
}

/// Returns the deterministic noise key for the onboard camera at `frame_index`.
fn city_camera_noise_key(frame_index: usize) -> SensorNoiseKey {
    SensorNoiseKey::new(
        CITY_WORLD_SEED,
        CAMERA_STREAM_ID,
        CAMERA_STREAM_ID,
        frame_index as u64 + 1,
    )
}

/// Returns the pose of the forward camera mounted behind the windshield.
///
/// The render view convention looks along local `-Z`, so the yaw is taken from the
/// direction opposite the vehicle heading, matching [`follow_camera`].
fn vehicle_camera_transform(vehicle: VehicleFrame) -> Transform3 {
    let forward = vehicle.transform.rotation * Vec3::X;
    let yaw = (-forward.x).atan2(-forward.z);
    let rotation =
        (Quat::from_rotation_y(yaw) * Quat::from_rotation_x(CAMERA_PITCH_RAD)).normalize();
    Transform3::from_translation_rotation(
        Vec3::new(
            vehicle.transform.translation.x,
            vehicle.class.height_m() * 0.86,
            vehicle.transform.translation.z,
        ) + forward * (vehicle.class.length_m() * 0.18),
        rotation,
    )
}

/// Captures the onboard camera deterministically without a GPU.
///
/// [`HeadlessRenderBackend`] resolves scene geometry through the shared depth probe
/// rather than rasterizing, so this capture is the GPU-free acceptance signal that
/// the camera is mounted, moving with the vehicle, and observing the city. The wgpu
/// path renders the same pose for the picture-in-picture insets.
fn capture_city_camera_frames(
    city_scene: &RenderScene,
    vehicle_assets: &VehicleRenderAssets,
    traffic_frames: &[Vec<VehicleFrame>],
    primary_actor_index: usize,
) -> CityCameraCapture {
    let spec = city_camera_spec();
    let mut render = HeadlessRenderBackend::new();
    let mut frames = Vec::with_capacity(traffic_frames.len());
    let mut stable_hash = 0xcbf29ce484222325_u64;
    let mut pixels_per_frame = 0_usize;
    let mut nearest_depth_m = f32::INFINITY;
    let mut center_depth_sum = 0.0_f64;
    let mut previous_pose: Option<Transform3> = None;

    for (frame_index, vehicles) in traffic_frames.iter().enumerate() {
        let primary = vehicles[primary_actor_index];
        let mut scene = city_scene.clone();
        append_city_fleet(
            &mut scene,
            vehicle_assets,
            vehicles,
            primary_actor_index,
            false,
        );
        let sample = sample_camera_rgbd_swept(
            &mut render,
            &city_camera_sweep(previous_pose, primary),
            &spec,
            SimTime::from_ticks(frame_index as u64),
            &scene,
            city_camera_noise_key(frame_index),
        );
        previous_pose = Some(vehicle_camera_transform(primary));

        pixels_per_frame = sample.depth.depth_m.len();
        let center_depth_m = sample.depth.center_depth_m();
        let min_depth_m = sample.depth.min_depth_m();
        nearest_depth_m = nearest_depth_m.min(min_depth_m);
        center_depth_sum += f64::from(center_depth_m);
        stable_hash = hash_camera_sample(stable_hash, &sample);
        frames.push(CityCameraFrame {
            center_depth_m,
            min_depth_m,
        });
    }

    CityCameraCapture {
        stable_hash,
        pixels_per_frame,
        nearest_depth_m: if nearest_depth_m.is_finite() {
            nearest_depth_m
        } else {
            0.0
        },
        mean_center_depth_m: if frames.is_empty() {
            0.0
        } else {
            center_depth_sum / frames.len() as f64
        },
        frames,
    }
}

fn hash_camera_sample(mut hash: u64, sample: &CameraRgbdSample) -> u64 {
    for byte in &sample.rgb.rgba8 {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash ^= sample.depth.hash_depth();
    hash.wrapping_mul(0x100000001b3)
}

fn lidar_plate_transform(vehicle: VehicleFrame) -> Transform3 {
    let backward = vehicle.transform.rotation * Vec3::NEG_X;
    Transform3::from_translation_rotation(
        Vec3::new(
            vehicle.transform.translation.x,
            vehicle.class.height_m() * 0.35,
            vehicle.transform.translation.z,
        ) + backward * (vehicle.class.length_m() * 0.5 + 0.03),
        vehicle.transform.rotation,
    )
}

fn lidar_mount_transform(vehicle: VehicleFrame) -> Transform3 {
    let forward = vehicle.transform.rotation * Vec3::X;
    Transform3::from_translation_rotation(
        Vec3::new(
            vehicle.transform.translation.x,
            vehicle.class.height_m() + 0.18,
            vehicle.transform.translation.z,
        ) + forward * 0.55,
        vehicle.transform.rotation,
    )
}

fn hash_lidar_cloud(mut hash: u64, cloud: &PointCloud) -> u64 {
    for point in &cloud.points_m {
        for value in [point.x, point.y, point.z] {
            for byte in value.to_bits().to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(0x100000001b3);
            }
        }
    }
    for intensity in &cloud.intensities {
        for byte in intensity.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    for value in &cloud.ray_indices {
        for byte in value.to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    for value in &cloud.return_indices {
        hash ^= u64::from(*value);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    for value in &cloud.channel_indices {
        for byte in value.to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    for value in &cloud.timestamps_s {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}

fn stable_name_hash(value: &str) -> u64 {
    value.bytes().fold(0xcbf29ce484222325_u64, |hash, byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
    })
}

fn flatten_buildings_to_road_datum(bundle: &mut SceneAssetBundle) {
    for object in &mut bundle.scene.objects {
        if !object.name.starts_with("plateau_building_") {
            continue;
        }
        let Some(SceneCollisionAsset::Box { size_m }) = object.collision else {
            continue;
        };
        object.translation_m[1] = size_m[1] * 0.5;
    }
}

fn append_lane_markings(scene: &mut RenderScene, lanes: &[ImportedLane]) {
    let start = (Vec3::from_array(lanes[0].centerline_m[0])
        + Vec3::from_array(lanes[1].centerline_m[1]))
        * 0.5;
    let end = (Vec3::from_array(lanes[0].centerline_m[1])
        + Vec3::from_array(lanes[1].centerline_m[0]))
        * 0.5;
    let direction = (end - start).normalize_or_zero();
    let length_m = (end - start).length();
    let yaw_rad = -direction.z.atan2(direction.x);
    let dash_length_m = 3.2;
    let dash_period_m = 7.0;
    let dash_count = (length_m / dash_period_m).floor() as usize;
    for index in 0..dash_count {
        let distance_m = index as f64 * dash_period_m + dash_length_m * 0.5;
        let center = start + direction * distance_m;
        push_box(
            scene,
            Vec3::new(center.x, 0.075, center.z),
            Quat::from_rotation_y(yaw_rad),
            Vec3::new(dash_length_m, 0.018, 0.11),
            [0.82, 0.80, 0.69, 1.0],
        );
    }
}

fn append_runtime_route_pavement(scene: &mut RenderScene, routes: &TrafficRouteCatalog) {
    let mut mesh = DebugMeshBuilder::default();
    for (_, route) in routes.iter() {
        mesh.add_polyline(route.path_m(), 0.058, 3.4);
    }
    push_debug_mesh(scene, mesh, [0.075, 0.085, 0.090, 1.0]);
}

fn append_traffic_debug_overlay(
    scene: &mut RenderScene,
    network: &TrafficNetwork,
    routes: &TrafficRouteCatalog,
    signal_route: &TrafficRoute,
    signal_distances_m: &[f64],
    overlay: TrafficDebugOverlay,
) {
    if overlay.lanes {
        let mut mesh = DebugMeshBuilder::default();
        for lane in &network.lanes {
            mesh.add_polyline(&lane.centerline_m, 0.18, 0.10);
        }
        push_debug_mesh(scene, mesh, [0.08, 0.52, 1.0, 1.0]);
    }
    if overlay.connections {
        let mut mesh = DebugMeshBuilder::default();
        for connection in &network.connections {
            mesh.add_polyline(&connection.path_m, 0.23, 0.08);
        }
        push_debug_mesh(scene, mesh, [0.08, 0.95, 0.72, 1.0]);
    }
    if overlay.route {
        const ROUTE_COLORS: [[f32; 4]; CITY_ROUTE_COUNT] = [
            [1.0, 0.72, 0.04, 1.0],
            [0.98, 0.24, 0.18, 1.0],
            [0.66, 0.28, 0.96, 1.0],
            [0.04, 0.78, 0.96, 1.0],
            [0.18, 0.90, 0.38, 1.0],
            [1.0, 0.38, 0.72, 1.0],
            [0.98, 0.92, 0.16, 1.0],
            [0.20, 0.52, 1.0, 1.0],
        ];
        for (index, (_, route)) in routes.iter().enumerate() {
            let mut mesh = DebugMeshBuilder::default();
            mesh.add_polyline(route.path_m(), 0.30 + index as f64 * 0.01, 0.24);
            push_debug_mesh(scene, mesh, ROUTE_COLORS[index % ROUTE_COLORS.len()]);
        }
    }
    if overlay.signals {
        let mut mesh = DebugMeshBuilder::default();
        for distance_m in signal_distances_m {
            let point = Vec3::from_array(signal_route.sample(*distance_m).position_m);
            mesh.add_marker(point, 0.52, 0.55);
        }
        push_debug_mesh(scene, mesh, [1.0, 0.12, 0.04, 1.0]);
    }
    if overlay.conflict_points {
        let mut mesh = DebugMeshBuilder::default();
        for (left_index, left) in network.connections.iter().enumerate() {
            for right_id in &left.conflict_connection_ids {
                let Some(right_index) = network
                    .connections
                    .iter()
                    .position(|connection| &connection.id == right_id)
                else {
                    continue;
                };
                if right_index <= left_index {
                    continue;
                }
                let point = closest_path_pair_midpoint(
                    &left.path_m,
                    &network.connections[right_index].path_m,
                );
                mesh.add_marker(point, 0.68, 0.34);
            }
        }
        push_debug_mesh(scene, mesh, [1.0, 0.02, 0.42, 1.0]);
    }
}

/// Number of quantized turbo-colormap buckets used for the point cloud.
const LIDAR_COLORMAP_BUCKETS: usize = 10;
/// Intensity mapped to the top of the colormap; retroreflective hits clip to red.
const LIDAR_COLORMAP_FULL_SCALE: f64 = 0.30;

/// Google's turbo colormap polynomial approximation.
///
/// Turbo is the perceptually even rainbow used by real point-cloud viewers
/// (Rerun, the KITTI/nuScenes toolchains): dim returns read as deep blue, mid
/// returns pass through green and orange, and saturated retroreflective hits
/// land on red, with no banding between them.
fn turbo_colormap(t: f64) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let polynomial = |c: [f64; 6]| -> f32 {
        (c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))).clamp(0.0, 1.0) as f32
    };
    [
        polynomial([
            0.135_721_38,
            4.615_392_60,
            -42.660_322_58,
            132.131_082_34,
            -152.942_393_96,
            59.286_379_43,
        ]),
        polynomial([
            0.091_402_61,
            2.194_188_39,
            4.842_966_58,
            -14.185_033_33,
            4.277_298_57,
            2.829_566_04,
        ]),
        polynomial([
            0.106_673_30,
            12.641_946_08,
            -60.582_048_36,
            110.362_767_71,
            -89.903_109_12,
            27.348_249_73,
        ]),
        1.0,
    ]
}

/// Draws the point cloud the way a real LiDAR viewer would.
///
/// Continuous turbo colormap over intensity instead of discrete bands, no drawn
/// beams (viewers show returns, not rays), and small markers that grow slightly
/// with range so distant rings survive perspective. The square-root intensity
/// normalization spreads the crowded low end of the radiometric distribution
/// across the palette.
fn append_lidar_intensity_overlay(scene: &mut RenderScene, frame: &CityLidarFrame) {
    let mut buckets: Vec<DebugMeshBuilder> = (0..LIDAR_COLORMAP_BUCKETS)
        .map(|_| DebugMeshBuilder::default())
        .collect();

    for (point, intensity) in frame
        .cloud
        .points_m
        .iter()
        .zip(frame.cloud.intensities.iter().copied())
    {
        let normalized = (f64::from(intensity) / LIDAR_COLORMAP_FULL_SCALE)
            .clamp(0.0, 1.0)
            .sqrt();
        let bucket = ((normalized * (LIDAR_COLORMAP_BUCKETS - 1) as f64).round() as usize)
            .min(LIDAR_COLORMAP_BUCKETS - 1);
        let range_m = (*point - frame.mount.translation).length();
        // World-space markers shrink with perspective; a mild range term keeps the
        // far rings visible the way constant-size screen points do in a viewer.
        let size = (0.075 + range_m * 0.002).min(0.16);
        buckets[bucket].add_point_marker(*point, size);
    }

    for (bucket, mesh) in buckets.into_iter().enumerate() {
        // Empty buckets would still cost a scene item; skip them.
        if mesh.positions.is_empty() {
            continue;
        }
        let t = bucket as f64 / (LIDAR_COLORMAP_BUCKETS - 1) as f64;
        push_debug_mesh(scene, mesh, turbo_colormap(t));
    }
}

#[derive(Debug, Default)]
struct DebugMeshBuilder {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl DebugMeshBuilder {
    fn add_polyline(&mut self, points_m: &[[f64; 3]], height_m: f64, width_m: f64) {
        for segment in points_m.windows(2) {
            let start = Vec3::from_array(segment[0]) + Vec3::new(0.0, height_m, 0.0);
            let end = Vec3::from_array(segment[1]) + Vec3::new(0.0, height_m, 0.0);
            let delta = end - start;
            if delta.x.hypot(delta.z) <= 1.0e-6 {
                continue;
            }
            let right = Vec3::new(-delta.z, 0.0, delta.x).normalize_or_zero() * width_m * 0.5;
            self.add_quad(start - right, start + right, end - right, end + right);
        }
    }

    fn add_marker(&mut self, point: Vec3, height_m: f64, radius_m: f64) {
        let center = point + Vec3::new(0.0, height_m, 0.0);
        self.add_quad(
            center + Vec3::new(-radius_m, 0.0, 0.0),
            center + Vec3::new(0.0, 0.0, -radius_m),
            center + Vec3::new(0.0, 0.0, radius_m),
            center + Vec3::new(radius_m, 0.0, 0.0),
        );
    }

    fn add_point_marker(&mut self, center: Vec3, radius_m: f64) {
        let x = Vec3::X * radius_m;
        let y = Vec3::Y * radius_m;
        let z = Vec3::Z * radius_m;
        self.add_quad(center - x, center + y, center - y, center + x);
        self.add_quad(center - z, center + y, center - y, center + z);
        self.add_quad(center - x, center + z, center - z, center + x);
    }

    fn add_quad(&mut self, first: Vec3, second: Vec3, third: Vec3, fourth: Vec3) {
        let base = self.positions.len() as u32;
        self.positions.extend(
            [first, second, third, fourth]
                .map(|point| [point.x as f32, point.y as f32, point.z as f32]),
        );
        self.normals.extend([[0.0, 1.0, 0.0]; 4]);
        self.texcoords.extend([[0.0, 0.0]; 4]);
        self.indices
            .extend([base, base + 1, base + 2, base + 2, base + 1, base + 3]);
    }
}

fn push_debug_mesh(scene: &mut RenderScene, mesh: DebugMeshBuilder, color_rgba: [f32; 4]) {
    if mesh.indices.is_empty() {
        return;
    }
    scene.items.push(RenderSceneItem {
        transform: MathTransform3::IDENTITY,
        shape: VisualShape::DynamicMesh,
        color_rgba,
        mesh: Some(Arc::new(TriangleMesh {
            positions: mesh.positions,
            normals: mesh.normals,
            texcoords: mesh.texcoords,
            indices: mesh.indices,
        })),
        base_color_texture: None,
    });
}

fn closest_path_pair_midpoint(left: &[[f64; 3]], right: &[[f64; 3]]) -> Vec3 {
    let mut best = (f64::INFINITY, Vec3::ZERO);
    for left_point in left {
        for right_point in right {
            let left_point = Vec3::from_array(*left_point);
            let right_point = Vec3::from_array(*right_point);
            let distance_squared_m = (right_point - left_point).length_squared();
            if distance_squared_m < best.0 {
                best = (distance_squared_m, (left_point + right_point) * 0.5);
            }
        }
    }
    best.1
}

#[cfg(test)]
fn write_facade_texture(path: &Path) {
    const SIZE: u32 = 384;
    let image = image::RgbaImage::from_fn(SIZE, SIZE, |x, y| {
        let mortar = x % 64 < 4 || y % 64 < 4;
        let inset_x = x % 64;
        let inset_y = y % 64;
        let window = (10..54).contains(&inset_x) && (12..50).contains(&inset_y);
        let pixel = if mortar {
            [132, 126, 116, 255]
        } else if window {
            let glint = ((x / 64 + y / 64) & 1) == 0;
            if glint {
                [72, 116, 142, 255]
            } else {
                [45, 78, 101, 255]
            }
        } else {
            [190, 181, 165, 255]
        };
        image::Rgba(pixel)
    });
    image
        .save(path)
        .expect("write procedural CC0 facade texture");
}

#[cfg(test)]
fn showcase_citygml() -> String {
    let mut xml = String::from(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!-- Synthetic CC0 PLATEAU-style showcase; contains no surveyed geometry. -->
<core:CityModel
    xmlns:core="http://www.opengis.net/citygml/2.0"
    xmlns:gml="http://www.opengis.net/gml"
    xmlns:bldg="http://www.opengis.net/citygml/building/2.0"
    xmlns:app="http://www.opengis.net/citygml/appearance/2.0"
    xmlns:tran="http://www.opengis.net/citygml/transportation/2.0">
  <gml:boundedBy>
    <gml:Envelope srsName="urn:ogc:def:crs:EPSG::6677" srsDimension="3">
      <gml:lowerCorner>-22 -45 0</gml:lowerCorner>
      <gml:upperCorner>22 45 24</gml:upperCorner>
    </gml:Envelope>
  </gml:boundedBy>
  <core:cityObjectMember>
    <tran:Road gml:id="showcase-avenue">
      <gml:name>Showcase Avenue</gml:name>
      <tran:function>1</tran:function>
      <tran:lod1MultiSurface>
        <gml:MultiSurface>
          <gml:surfaceMember>
            <gml:Polygon>
              <gml:exterior><gml:LinearRing>
                <gml:posList srsDimension="3">-4 45 0 -4 -45 0 4 -45 0 4 45 0 -4 45 0</gml:posList>
              </gml:LinearRing></gml:exterior>
            </gml:Polygon>
          </gml:surfaceMember>
        </gml:MultiSurface>
      </tran:lod1MultiSurface>
    </tran:Road>
  </core:cityObjectMember>
"#,
    );
    for building in SHOWCASE_BUILDINGS {
        append_showcase_building(&mut xml, building);
    }
    xml.push_str(
        "  <app:appearanceMember>\n    <app:Appearance>\n      <app:theme>rgbTexture</app:theme>\n      <app:surfaceDataMember>\n        <app:ParameterizedTexture>\n          <app:imageURI>appearance/facade.png</app:imageURI>\n          <app:mimeType>image/png</app:mimeType>\n",
    );
    for building in SHOWCASE_BUILDINGS {
        for surface_index in 2..6 {
            xml.push_str(&format!(
                "          <app:target uri=\"#{}-polygon-{surface_index}\"><app:TexCoordList><app:textureCoordinates ring=\"#{}-ring-{surface_index}\">0 0 1 0 1 1 0 1 0 0</app:textureCoordinates></app:TexCoordList></app:target>\n",
                building.id, building.id
            ));
        }
    }
    xml.push_str(
        "        </app:ParameterizedTexture>\n      </app:surfaceDataMember>\n    </app:Appearance>\n  </app:appearanceMember>\n",
    );
    xml.push_str("</core:CityModel>\n");
    xml
}

#[cfg(test)]
fn append_showcase_building(xml: &mut String, building: ShowcaseBuilding) {
    let x0 = building.x_min_m;
    let x1 = building.x_max_m;
    let s0 = -building.z_min_m;
    let s1 = -building.z_max_m;
    let h = building.height_m;
    let rings = [
        format!("{x0} {s0} 0 {x1} {s0} 0 {x1} {s1} 0 {x0} {s1} 0 {x0} {s0} 0"),
        format!("{x0} {s0} {h} {x0} {s1} {h} {x1} {s1} {h} {x1} {s0} {h} {x0} {s0} {h}"),
        format!("{x0} {s0} 0 {x0} {s0} {h} {x1} {s0} {h} {x1} {s0} 0 {x0} {s0} 0"),
        format!("{x1} {s0} 0 {x1} {s0} {h} {x1} {s1} {h} {x1} {s1} 0 {x1} {s0} 0"),
        format!("{x1} {s1} 0 {x1} {s1} {h} {x0} {s1} {h} {x0} {s1} 0 {x1} {s1} 0"),
        format!("{x0} {s1} 0 {x0} {s1} {h} {x0} {s0} {h} {x0} {s0} 0 {x0} {s1} 0"),
    ];
    xml.push_str(&format!(
        "  <core:cityObjectMember>\n    <bldg:Building gml:id=\"{}\">\n      <gml:name>{}</gml:name>\n      <bldg:function>401</bldg:function>\n      <bldg:measuredHeight uom=\"m\">{h}</bldg:measuredHeight>\n",
        building.id, building.name
    ));
    let surface_types = [
        "GroundSurface",
        "RoofSurface",
        "WallSurface",
        "WallSurface",
        "WallSurface",
        "WallSurface",
    ];
    for (surface_index, (ring, surface_type)) in rings.into_iter().zip(surface_types).enumerate() {
        xml.push_str(&format!(
            "      <bldg:boundedBy><bldg:{surface_type} gml:id=\"{}-surface-{surface_index}\"><bldg:lod2MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon gml:id=\"{}-polygon-{surface_index}\"><gml:exterior><gml:LinearRing gml:id=\"{}-ring-{surface_index}\"><gml:posList srsDimension=\"3\">{ring}</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></bldg:lod2MultiSurface></bldg:{surface_type}></bldg:boundedBy>\n",
            building.id, building.id, building.id
        ));
    }
    xml.push_str("    </bldg:Building>\n  </core:cityObjectMember>\n");
}

fn render_scene_from_world(world: &mut World) -> RenderScene {
    let mut scene = RenderScene::new();
    let mut query = world.query::<(&Transform3, &Visual)>();
    for (transform, visual) in query.iter(world) {
        scene.items.push(RenderScene::item_from_visual(
            *transform,
            visual.shape.clone(),
            visual.color_rgba,
            visual.local_offset,
        ));
    }
    scene
}

fn building_footprints(bundle: &SceneAssetBundle) -> Vec<Footprint> {
    bundle
        .scene
        .objects
        .iter()
        .filter(|object| object.name.starts_with("plateau_building_"))
        .filter_map(|object| {
            let SceneCollisionAsset::Box { size_m } = object.collision? else {
                return None;
            };
            Some(Footprint {
                min_x_m: object.translation_m[0] - size_m[0] * 0.5,
                max_x_m: object.translation_m[0] + size_m[0] * 0.5,
                min_z_m: object.translation_m[2] - size_m[2] * 0.5,
                max_z_m: object.translation_m[2] + size_m[2] * 0.5,
            })
        })
        .collect()
}

fn append_city_streetscape(
    scene: &mut RenderScene,
    lanes: &[ImportedLane],
    buildings: &[Footprint],
) -> Vec<StreetFixture> {
    let road = RoadFrame::from_lanes(lanes);
    let rotation = Quat::from_rotation_y(road.yaw_rad);
    let mut fixtures = Vec::new();

    // A low ground slab receives shadows and keeps non-road pixels from becoming sky.
    push_box(
        scene,
        Vec3::new(0.0, -0.12, 0.0),
        Quat::IDENTITY,
        Vec3::new(360.0, 0.20, 360.0),
        [0.21, 0.27, 0.19, 1.0],
    );

    let sidewalk_width_m = 2.4;
    let sidewalk_center_offset_m = road.half_width_m + sidewalk_width_m * 0.5;
    for side in [-1.0, 1.0] {
        let sidewalk_center = road.point(0.0, side * sidewalk_center_offset_m, 0.055);
        push_box(
            scene,
            sidewalk_center,
            rotation,
            Vec3::new(road.length_m + 3.0, 0.11, sidewalk_width_m),
            [0.48, 0.49, 0.47, 1.0],
        );
        let curb_center = road.point(0.0, side * (road.half_width_m + 0.08), 0.105);
        push_box(
            scene,
            curb_center,
            rotation,
            Vec3::new(road.length_m + 3.0, 0.21, 0.16),
            [0.67, 0.67, 0.63, 1.0],
        );

        for along_fraction in [-0.38, -0.12, 0.15, 0.40] {
            let base = road.point(
                road.length_m * along_fraction,
                side * (road.half_width_m + 1.55),
                0.11,
            );
            if fixture_is_clear(base, 0.45, buildings) {
                append_streetlight(scene, base, -side, rotation);
                fixtures.push(StreetFixture {
                    center: base,
                    clearance_radius_m: 0.45,
                });
            }
        }

        for along_fraction in [-0.31, -0.02, 0.27] {
            let base = road.point(
                road.length_m * along_fraction,
                side * (road.half_width_m + 3.0),
                0.0,
            );
            if fixture_is_clear(base, 1.15, buildings) {
                append_tree(scene, base);
                fixtures.push(StreetFixture {
                    center: base,
                    clearance_radius_m: 1.15,
                });
            }
        }

        for along_fraction in [-0.24, 0.30] {
            let center = road.point(
                road.length_m * along_fraction,
                side * (road.half_width_m + 0.58),
                0.12,
            );
            if fixture_is_clear(center, 3.2, buildings) {
                append_guardrail(scene, center, rotation);
                fixtures.push(StreetFixture {
                    center,
                    clearance_radius_m: 3.2,
                });
            }
        }
    }
    fixtures
}

fn fixture_is_clear(center: Vec3, radius_m: f64, buildings: &[Footprint]) -> bool {
    buildings
        .iter()
        .all(|building| !building.overlaps_disc(center, radius_m))
}

fn append_streetlight(
    scene: &mut RenderScene,
    base: Vec3,
    road_direction: f64,
    road_rotation: Quat,
) {
    push_cylinder(
        scene,
        base + Vec3::new(0.0, 2.6, 0.0),
        Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        0.09,
        5.2,
        [0.18, 0.20, 0.21, 1.0],
    );
    push_box(
        scene,
        base + road_rotation * Vec3::new(0.0, 5.12, road_direction * 0.55),
        road_rotation,
        Vec3::new(0.08, 0.08, 1.1),
        [0.18, 0.20, 0.21, 1.0],
    );
    push_box(
        scene,
        base + road_rotation * Vec3::new(0.0, 5.02, road_direction * 1.05),
        road_rotation,
        Vec3::new(0.24, 0.13, 0.48),
        [0.28, 0.30, 0.30, 1.0],
    );
    push_box(
        scene,
        base + road_rotation * Vec3::new(0.0, 4.94, road_direction * 1.05),
        road_rotation,
        Vec3::new(0.16, 0.04, 0.34),
        [0.92, 0.82, 0.50, 1.0],
    );
}

fn append_tree(scene: &mut RenderScene, base: Vec3) {
    push_cylinder(
        scene,
        base + Vec3::new(0.0, 1.25, 0.0),
        Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        0.18,
        2.5,
        [0.27, 0.15, 0.07, 1.0],
    );
    for (offset, radius_m, color) in [
        (Vec3::new(-0.42, 3.05, 0.08), 0.88, [0.10, 0.29, 0.13, 1.0]),
        (Vec3::new(0.38, 3.15, -0.18), 0.94, [0.13, 0.37, 0.17, 1.0]),
        (Vec3::new(0.02, 3.74, 0.12), 0.82, [0.16, 0.42, 0.19, 1.0]),
    ] {
        push_sphere(scene, base + offset, radius_m, color);
    }
}

fn append_guardrail(scene: &mut RenderScene, center: Vec3, road_rotation: Quat) {
    for height_m in [0.42, 0.78] {
        push_box(
            scene,
            center + Vec3::new(0.0, height_m, 0.0),
            road_rotation,
            Vec3::new(5.8, 0.10, 0.10),
            [0.58, 0.60, 0.58, 1.0],
        );
    }
    for along_m in [-2.7, -0.9, 0.9, 2.7] {
        let post = center + road_rotation * Vec3::new(along_m, 0.39, 0.0);
        push_box(
            scene,
            post,
            road_rotation,
            Vec3::new(0.10, 0.78, 0.10),
            [0.42, 0.44, 0.43, 1.0],
        );
    }
}

fn append_intersection_markings(scene: &mut RenderScene, route: &TurnRoute) {
    let rotation =
        Quat::from_rotation_y(-route.incoming_direction.z.atan2(route.incoming_direction.x));
    push_box(
        scene,
        route.intersection + Vec3::new(0.0, 0.035, 0.0),
        rotation,
        Vec3::new(
            route.incoming_half_width_m * 2.05,
            0.045,
            route.incoming_half_width_m * 2.05,
        ),
        [0.135, 0.155, 0.166, 1.0],
    );
    push_box(
        scene,
        route.stop_point + Vec3::new(0.0, 0.075, 0.0),
        rotation,
        Vec3::new(0.42, 0.025, route.incoming_half_width_m * 1.75),
        [0.92, 0.92, 0.88, 1.0],
    );
    for index in 0..5 {
        let center = route.stop_point + route.incoming_direction * (1.2 + index as f64 * 0.75);
        push_box(
            scene,
            center + Vec3::new(0.0, 0.077, 0.0),
            rotation,
            Vec3::new(0.38, 0.026, route.incoming_half_width_m * 1.65),
            [0.88, 0.88, 0.84, 1.0],
        );
    }
}

fn append_city_runtime_signals(
    scene: &mut RenderScene,
    route: &TrafficRoute,
    signal_distances_m: &[f64],
    step: usize,
) {
    for (index, distance_m) in signal_distances_m.iter().enumerate() {
        let sample = route.sample(*distance_m);
        let rotation = Quat::from_rotation_y(sample.yaw_rad);
        let right = rotation * Vec3::Z;
        let base = Vec3::from_array(sample.position_m) + right * 3.8;
        let phase = match city_signal_aspect(step as u64, index) {
            SignalAspect::Green => SignalPhase::Green,
            _ => SignalPhase::Red,
        };
        append_traffic_signal(scene, base, -1.0, rotation, phase);
    }
}

fn append_traffic_signal(
    scene: &mut RenderScene,
    base: Vec3,
    road_direction: f64,
    road_rotation: Quat,
    phase: SignalPhase,
) {
    push_cylinder(
        scene,
        base + Vec3::new(0.0, 2.45, 0.0),
        Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        0.10,
        4.9,
        [0.15, 0.17, 0.17, 1.0],
    );
    push_box(
        scene,
        base + road_rotation * Vec3::new(0.0, 4.78, road_direction * 0.72),
        road_rotation,
        Vec3::new(0.10, 0.10, 1.45),
        [0.15, 0.17, 0.17, 1.0],
    );
    let housing = base + road_rotation * Vec3::new(0.0, 4.56, road_direction * 1.36);
    push_box(
        scene,
        housing,
        road_rotation,
        Vec3::new(0.38, 0.82, 0.40),
        [0.035, 0.045, 0.045, 1.0],
    );
    let red_color = if phase == SignalPhase::Red {
        [1.0, 0.025, 0.012, 1.0]
    } else {
        [0.12, 0.008, 0.005, 1.0]
    };
    let green_color = if phase == SignalPhase::Green {
        [0.015, 0.95, 0.18, 1.0]
    } else {
        [0.004, 0.12, 0.018, 1.0]
    };
    let lens_offset = road_rotation * Vec3::new(0.0, 0.0, road_direction * 0.215);
    push_sphere(
        scene,
        housing + Vec3::new(0.0, 0.21, 0.0) + lens_offset,
        0.105,
        red_color,
    );
    push_sphere(
        scene,
        housing + Vec3::new(0.0, -0.21, 0.0) + lens_offset,
        0.105,
        green_color,
    );
}

#[cfg(test)]
fn drone_position(progress: f64) -> Vec3 {
    let x = -20.0 + 40.0 * progress;
    let z = 8.5 - 17.0 * progress;
    let y = 14.5 + (progress * std::f64::consts::TAU).sin();
    Vec3::new(x, y, z)
}

fn select_station_road_lanes(lanes: &[ImportedLane]) -> Vec<ImportedLane> {
    let selected = lanes
        .iter()
        .filter(|lane| lane.lane_id.ends_with("/lane-0"))
        .filter(|lane| lane_length_m(lane) >= 45.0)
        .min_by(|left, right| {
            lane_distance_to_station_m(left).total_cmp(&lane_distance_to_station_m(right))
        })
        .expect("official tile must contain a long road near Kita-Sanjo Station");
    let surface_id = selected
        .lane_id
        .strip_suffix("/lane-0")
        .expect("lane-0 suffix");
    let opposing_id = format!("{surface_id}/lane-1");
    let opposing = lanes
        .iter()
        .find(|lane| lane.lane_id == opposing_id)
        .expect("derived lane must have its opposing lane");
    vec![selected.clone(), opposing.clone()]
}

fn select_station_turn_lane(lanes: &[ImportedLane], incoming: &ImportedLane) -> ImportedLane {
    let incoming_direction = lane_direction(incoming);
    let incoming_end = Vec3::from_array(incoming.centerline_m[1]);
    lanes
        .iter()
        .filter(|lane| lane.lane_id.ends_with("/lane-0"))
        .filter(|lane| lane.road_source_id != incoming.road_source_id)
        .filter(|lane| lane_length_m(lane) >= 35.0)
        .filter(|lane| lane_direction(lane).dot(incoming_direction).abs() < 0.50)
        .filter(|lane| (Vec3::from_array(lane.centerline_m[0]) - incoming_end).length() <= 5.0)
        .min_by(|left, right| {
            let left_distance =
                (Vec3::from_array(left.centerline_m[0]) - incoming_end).length_squared();
            let right_distance =
                (Vec3::from_array(right.centerline_m[0]) - incoming_end).length_squared();
            left_distance.total_cmp(&right_distance)
        })
        .cloned()
        .expect("official tile must contain a perpendicular outgoing road at the station junction")
}

fn lane_length_m(lane: &ImportedLane) -> f64 {
    let start = Vec3::from_array(lane.centerline_m[0]);
    let end = Vec3::from_array(lane.centerline_m[1]);
    (end - start).length()
}

fn lane_direction(lane: &ImportedLane) -> Vec3 {
    (Vec3::from_array(lane.centerline_m[1]) - Vec3::from_array(lane.centerline_m[0]))
        .normalize_or_zero()
}

fn lane_distance_to_station_m(lane: &ImportedLane) -> f64 {
    let start = Vec3::from_array(lane.centerline_m[0]);
    let end = Vec3::from_array(lane.centerline_m[1]);
    let station = Vec3::new(KITA_SANJO_STATION_XZ_M[0], 0.05, KITA_SANJO_STATION_XZ_M[1]);
    let segment = end - start;
    let progress = ((station - start).dot(segment) / segment.length_squared()).clamp(0.0, 1.0);
    (station - (start + segment * progress)).length()
}

#[cfg(test)]
fn simulate_two_way_traffic(
    lanes: &[ImportedLane],
    frame_count: usize,
) -> (Vec<VehicleFrame>, Vec<VehicleFrame>) {
    assert_eq!(lanes.len(), 2, "example requires one derived two-way road");
    let mut ordered: Vec<&ImportedLane> = lanes.iter().collect();
    ordered.sort_by(|left, right| {
        let left_delta = left.centerline_m[1][2] - left.centerline_m[0][2];
        let right_delta = right.centerline_m[1][2] - right.centerline_m[0][2];
        right_delta.total_cmp(&left_delta)
    });
    (
        simulate_lane_vehicle(ordered[0], frame_count, 0.0),
        simulate_lane_vehicle(ordered[1], frame_count, 0.9),
    )
}

#[cfg(test)]
fn simulate_signalized_turn(
    lanes: &[ImportedLane],
    outgoing: &ImportedLane,
    frame_count: usize,
) -> (Vec<VehicleFrame>, Vec<VehicleFrame>, TurnRoute) {
    assert_eq!(lanes.len(), 2, "example requires one derived two-way road");
    let route = build_turn_route(&lanes[0], outgoing);
    let primary = simulate_route_vehicle(&route, frame_count);
    let opposing = simulate_lane_vehicle(&lanes[1], frame_count, 0.9);
    (primary, opposing, route)
}

fn build_turn_route(incoming: &ImportedLane, outgoing: &ImportedLane) -> TurnRoute {
    let incoming_direction = lane_direction(incoming);
    let outgoing_direction = lane_direction(outgoing);
    let incoming_end = Vec3::from_array(incoming.centerline_m[1]);
    let outgoing_start = Vec3::from_array(outgoing.centerline_m[0]);
    let intersection = line_intersection_xz(
        incoming_end,
        incoming_direction,
        outgoing_start,
        outgoing_direction,
    )
    .unwrap_or((incoming_end + outgoing_start) * 0.5);
    let radius_m = 8.0;
    let entry = intersection - incoming_direction * radius_m;
    let exit = intersection + outgoing_direction * radius_m;
    let route_start = entry - incoming_direction * 25.0;
    let route_end = exit + outgoing_direction * 34.0;
    let stop_point = entry - incoming_direction * 3.0;
    let mut points = Vec::new();
    append_sampled_line(&mut points, route_start, entry, 1.0);
    for index in 1..=16 {
        let t = index as f64 / 16.0;
        let one_minus_t = 1.0 - t;
        points.push(
            entry * one_minus_t.powi(2) + intersection * (2.0 * one_minus_t * t) + exit * t.powi(2),
        );
    }
    append_sampled_line(&mut points, exit, route_end, 1.0);
    TurnRoute {
        points,
        intersection,
        incoming_direction,
        outgoing_direction,
        incoming_half_width_m: incoming.width_m,
        stop_point,
    }
}

fn line_intersection_xz(
    first_point: Vec3,
    first_direction: Vec3,
    second_point: Vec3,
    second_direction: Vec3,
) -> Option<Vec3> {
    let denominator =
        first_direction.x * second_direction.z - first_direction.z * second_direction.x;
    if denominator.abs() < 1.0e-6 {
        return None;
    }
    let delta = second_point - first_point;
    let progress = (delta.x * second_direction.z - delta.z * second_direction.x) / denominator;
    Some(first_point + first_direction * progress)
}

fn append_sampled_line(points: &mut Vec<Vec3>, start: Vec3, end: Vec3, spacing_m: f64) {
    let distance_m = (end - start).length();
    let sample_count = (distance_m / spacing_m).ceil().max(1.0) as usize;
    for index in 0..sample_count {
        let t = index as f64 / sample_count as f64;
        let point = start + (end - start) * t;
        if points
            .last()
            .is_none_or(|previous| (*previous - point).length_squared() > 1.0e-8)
        {
            points.push(point);
        }
    }
    points.push(end);
}

#[cfg(test)]
fn simulate_route_vehicle(route: &TurnRoute, frame_count: usize) -> Vec<VehicleFrame> {
    let fixed_delta = SimDuration::from_hertz(Hertz::new(SIM_HZ as f64));
    let mut clock = SimClock::new(fixed_delta);
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, "vehicle_signalized_turn");
    let route_id = TrafficId::new("plateau:sanjo/showcase-turn").expect("showcase route ID");
    let traffic_route = TrafficRoute::new(
        route_id.clone(),
        route
            .points
            .iter()
            .map(|point| [point.x, point.y + 0.60, point.z])
            .collect(),
        false,
    )
    .expect("showcase traffic route");
    let stop_distance_m = nearest_route_distance_m(&traffic_route, route.stop_point);
    let initial = traffic_route.sample(0.0);
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(traffic_route).expect("insert showcase route");
    world.entity_mut(vehicle).insert((
        TrafficActor::motor_vehicle(),
        TrafficRouteFollower {
            route_id,
            distance_m: 0.0,
            speed_m_s: 0.0,
            desired_speed_m_s: 5.2,
            length_m: 4.2,
        },
        TrafficPose {
            position_m: initial.position_m,
            yaw_rad: initial.yaw_rad,
        },
    ));
    let config = KinematicTrafficConfig {
        max_acceleration_m_s2: 2.2,
        max_braking_m_s2: 4.5,
        ..KinematicTrafficConfig::default()
    };
    let mut runtime = TrafficRuntime::default();
    let mut wheel_rotation_rad = 0.0;
    let mut frames = Vec::with_capacity(frame_count);
    for _ in 0..frame_count {
        let pose = *world.get::<TrafficPose>(vehicle).expect("traffic pose");
        let follower = world
            .get::<TrafficRouteFollower>(vehicle)
            .expect("route follower");
        let steering_rad = route_steering_rad(
            routes.get(&follower.route_id).expect("showcase route"),
            follower.distance_m,
            pose.yaw_rad,
        );
        frames.push(VehicleFrame {
            transform: Transform3::from_translation_rotation(
                Vec3::from_array(pose.position_m),
                Quat::from_rotation_y(pose.yaw_rad),
            ),
            speed_m_s: follower.speed_m_s,
            steering_rad,
            wheel_rotation_rad,
            braking: follower.speed_m_s > 0.2
                && follower.desired_speed_m_s + 0.05 < follower.speed_m_s,
            class: CityVehicleClass::Sedan,
        });
        for _ in 0..SIM_STEPS_PER_FRAME {
            let follower = world
                .get::<TrafficRouteFollower>(vehicle)
                .expect("route follower");
            let before_stop = follower.distance_m < stop_distance_m;
            let remaining_stop_m = (stop_distance_m - follower.distance_m - 0.6).max(0.0);
            let signal_speed_m_s = (2.0 * config.max_braking_m_s2 * remaining_stop_m).sqrt();
            let desired_speed_m_s = if before_stop
                && signal_phase_at(clock.sim_time().as_seconds().value()) == SignalPhase::Red
            {
                signal_speed_m_s.min(5.2)
            } else {
                5.2
            };
            world
                .get_mut::<TrafficRouteFollower>(vehicle)
                .expect("route follower")
                .desired_speed_m_s = desired_speed_m_s;
            assert_eq!(clock.advance(fixed_delta), 1);
            let speed_m_s = world
                .get::<TrafficRouteFollower>(vehicle)
                .expect("route follower")
                .speed_m_s;
            advance_kinematic_traffic(
                &mut world,
                &routes,
                &mut runtime,
                clock.sim_time(),
                fixed_delta,
                config,
            )
            .expect("advance showcase traffic");
            wheel_rotation_rad += speed_m_s * fixed_delta.as_seconds().value() / 0.36;
        }
    }
    frames
}

fn nearest_route_distance_m(route: &TrafficRoute, target: Vec3) -> f64 {
    let mut best = (f64::INFINITY, 0.0);
    let mut cumulative_m = 0.0;
    for segment in route.path_m().windows(2) {
        let start = Vec3::from_array(segment[0]);
        let end = Vec3::from_array(segment[1]);
        let delta = end - start;
        let length_m = delta.length();
        let progress = if length_m <= 1.0e-9 {
            0.0
        } else {
            ((target - start).dot(delta) / delta.length_squared()).clamp(0.0, 1.0)
        };
        let projected = start + delta * progress;
        let distance_squared_m = (target - projected).length_squared();
        if distance_squared_m < best.0 {
            best = (distance_squared_m, cumulative_m + length_m * progress);
        }
        cumulative_m += length_m;
    }
    best.1
}

fn route_steering_rad(route: &TrafficRoute, distance_m: f64, yaw_rad: f64) -> f64 {
    let lookahead_m = 4.5;
    let lookahead_yaw_rad = route.sample(distance_m + lookahead_m).yaw_rad;
    let heading_delta_rad = (lookahead_yaw_rad - yaw_rad + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    (2.7 * heading_delta_rad / lookahead_m)
        .atan()
        .clamp(-0.55, 0.55)
}

#[cfg(test)]
fn simulate_lane_vehicle(
    lane: &ImportedLane,
    frame_count: usize,
    start_delay_s: f64,
) -> Vec<VehicleFrame> {
    let fixed_delta = SimDuration::from_hertz(Hertz::new(SIM_HZ as f64));
    let mut clock = SimClock::new(fixed_delta);
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, format!("vehicle_{}", lane.lane_id));
    let start = Vec3::from_array(lane.centerline_m[0]) + Vec3::new(0.0, 0.65, 0.0);
    let end = Vec3::from_array(lane.centerline_m[1]) + Vec3::new(0.0, 0.65, 0.0);
    let direction = (end - start).normalize_or_zero();
    let yaw_rad = -direction.z.atan2(direction.x);
    let drive = AckermannDrive {
        max_speed_m_s: 7.0,
        max_acceleration_m_s2: 2.2,
        max_deceleration_m_s2: 4.5,
        max_steering_rate_rad_s: 0.7,
        ..AckermannDrive::default()
    };
    world.entity_mut(vehicle).insert((
        Transform3::from_translation_rotation(start, Quat::from_rotation_y(yaw_rad)),
        drive,
    ));
    let mut wheel_rotation_rad = 0.0;
    let mut frames = Vec::with_capacity(frame_count);
    for _ in 0..frame_count {
        let transform = *world.get::<Transform3>(vehicle).expect("vehicle transform");
        let drive = world
            .get::<AckermannDrive>(vehicle)
            .expect("Ackermann drive");
        frames.push(VehicleFrame {
            transform,
            speed_m_s: drive.speed_m_s,
            steering_rad: drive.steering_rad,
            wheel_rotation_rad,
            braking: drive.speed_m_s > 0.2 && drive.target_speed_m_s + 0.05 < drive.speed_m_s,
            class: CityVehicleClass::Sedan,
        });
        for _ in 0..SIM_STEPS_PER_FRAME {
            let transform = *world.get::<Transform3>(vehicle).expect("vehicle transform");
            let drive = world
                .get::<AckermannDrive>(vehicle)
                .expect("Ackermann drive");
            let remaining_m = (end - transform.translation).dot(direction).max(0.0);
            let stopping_speed_m_s = (2.0 * drive.max_deceleration_m_s2 * remaining_m).sqrt();
            let target_speed_m_s = if clock.sim_time().as_seconds().value() < start_delay_s {
                0.0
            } else {
                6.0_f64.min(stopping_speed_m_s)
            };
            let steering_rad = pure_pursuit_steering(&transform, end, drive.wheelbase_m, 6.0);
            let _ = command_ackermann_drive(&mut world, vehicle, target_speed_m_s, steering_rad);
            assert_eq!(clock.advance(fixed_delta), 1);
            ackermann_kinematics(&mut world, clock.fixed_delta());
            let passed_endpoint = {
                let transform = world.get::<Transform3>(vehicle).expect("vehicle transform");
                (end - transform.translation).dot(direction) <= 0.0
            };
            if passed_endpoint {
                let mut transform = world
                    .get_mut::<Transform3>(vehicle)
                    .expect("vehicle transform");
                transform.translation.x = end.x;
                transform.translation.z = end.z;
                let mut drive = world
                    .get_mut::<AckermannDrive>(vehicle)
                    .expect("Ackermann drive");
                drive.speed_m_s = 0.0;
                drive.target_speed_m_s = 0.0;
                drive.steering_rad = 0.0;
                drive.target_steering_rad = 0.0;
            }
            let speed_m_s = world
                .get::<AckermannDrive>(vehicle)
                .expect("Ackermann drive")
                .speed_m_s;
            wheel_rotation_rad += speed_m_s * fixed_delta.as_seconds().value() / 0.36;
        }
    }
    frames
}

fn follow_camera(vehicle: VehicleFrame) -> CameraOrbit {
    let forward = vehicle.transform.rotation * Vec3::X;
    let right = vehicle.transform.rotation * Vec3::Z;
    let eye_direction = (-forward + right * 0.10).normalize_or_zero();
    CameraOrbit {
        focus: vehicle.transform.translation + forward * 5.4 + Vec3::new(0.0, 0.42, 0.0),
        yaw_rad: eye_direction.x.atan2(eye_direction.z),
        pitch_rad: 1.40,
        distance_m: 11.8,
    }
}

fn append_city_fleet(
    scene: &mut RenderScene,
    assets: &VehicleRenderAssets,
    vehicles: &[VehicleFrame],
    detailed_actor_index: usize,
    detailed_lead: bool,
) {
    for (index, vehicle) in vehicles.iter().copied().enumerate() {
        let texture = if index % 3 == 0 {
            &assets.red_body_texture
        } else {
            &assets.blue_body_texture
        };
        if index == detailed_actor_index && detailed_lead {
            append_car(scene, assets, vehicle, texture);
        } else {
            append_fleet_body(scene, assets, vehicle, texture);
        }
    }
}

fn append_fleet_body(
    scene: &mut RenderScene,
    assets: &VehicleRenderAssets,
    vehicle: VehicleFrame,
    body_texture: &Arc<ImageFrame>,
) {
    let rotation = vehicle.transform.rotation;
    let mesh = assets
        .body_meshes
        .first()
        .expect("validated Kenney body mesh");
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation: vehicle.transform.translation + rotation * Vec3::new(0.0, -0.50, 0.0),
            rotation: rotation * Quat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            scale: vehicle.class.render_scale(),
        },
        shape: VisualShape::Mesh {
            path: "kenney://sedan-body-lod".into(),
            scale: Vec3::ONE,
        },
        color_rgba: [1.0; 4],
        mesh: Some(Arc::clone(mesh)),
        base_color_texture: Some(Arc::clone(body_texture)),
    });
}

fn append_car(
    scene: &mut RenderScene,
    assets: &VehicleRenderAssets,
    vehicle: VehicleFrame,
    body_texture: &Arc<ImageFrame>,
) {
    let center = vehicle.transform.translation;
    let rotation = vehicle.transform.rotation;
    append_vehicle_meshes(
        scene,
        &assets.body_meshes,
        body_texture,
        "kenney://sedan-body",
        MathTransform3 {
            translation: center + rotation * Vec3::new(0.0, -0.50, 0.0),
            rotation: rotation * Quat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            scale: Vec3::new(1.24, 1.25, 1.70),
        },
    );

    for (x_m, z_m, steerable) in [
        (-1.27, -0.82, false),
        (-1.27, 0.82, false),
        (1.32, -0.82, true),
        (1.32, 0.82, true),
    ] {
        let steering = if steerable { vehicle.steering_rad } else { 0.0 };
        append_vehicle_meshes(
            scene,
            &assets.wheel_meshes,
            &assets.wheel_texture,
            "kenney://wheel-default",
            MathTransform3 {
                translation: center + rotation * Vec3::new(x_m, -0.24, z_m),
                rotation: rotation
                    * Quat::from_rotation_y(steering)
                    * Quat::from_rotation_y(std::f64::consts::FRAC_PI_2)
                    * Quat::from_rotation_x(-vehicle.wheel_rotation_rad),
                scale: Vec3::new(0.65, 1.20, 1.20),
            },
        );
    }

    let brake_color = if vehicle.braking {
        [1.0, 0.025, 0.008, 1.0]
    } else {
        [0.24, 0.008, 0.004, 1.0]
    };
    for z_m in [-0.58, 0.58] {
        push_box(
            scene,
            center + rotation * Vec3::new(-2.19, 0.10, z_m),
            rotation,
            Vec3::new(0.055, 0.18, 0.32),
            brake_color,
        );
    }
    push_box(
        scene,
        center + rotation * Vec3::new(-2.19, -0.19, 0.0),
        rotation,
        Vec3::new(0.045, 0.14, 0.34),
        [0.78, 0.80, 0.78, 1.0],
    );
}

fn append_vehicle_meshes(
    scene: &mut RenderScene,
    meshes: &[Arc<TriangleMesh>],
    texture: &Arc<ImageFrame>,
    asset_id: &str,
    transform: MathTransform3,
) {
    for mesh in meshes {
        scene.items.push(RenderSceneItem {
            transform,
            shape: VisualShape::Mesh {
                path: asset_id.to_string(),
                scale: Vec3::ONE,
            },
            color_rgba: [1.0; 4],
            mesh: Some(Arc::clone(mesh)),
            base_color_texture: Some(Arc::clone(texture)),
        });
    }
}

fn push_box(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    size_m: Vec3,
    color_rgba: [f32; 4],
) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation,
            scale: size_m,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba,
        mesh: None,
        base_color_texture: None,
    });
}

fn push_cylinder(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    radius_m: f64,
    length_m: f64,
    color_rgba: [f32; 4],
) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation,
            scale: Vec3::new(radius_m * 2.0, radius_m * 2.0, length_m),
        },
        shape: VisualShape::Cylinder { radius_m, length_m },
        color_rgba,
        mesh: None,
        base_color_texture: None,
    });
}

fn push_sphere(scene: &mut RenderScene, translation: Vec3, radius_m: f64, color_rgba: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(radius_m * 2.0),
        },
        shape: VisualShape::Sphere { radius_m },
        color_rgba,
        mesh: None,
        base_color_texture: None,
    });
}

fn cinematic_postprocess(
    rgba8: &[u8],
    depth_m: &[f32],
    width: u32,
    height: u32,
    far_m: f32,
) -> Vec<u8> {
    assert_eq!(rgba8.len(), width as usize * height as usize * 4);
    assert_eq!(depth_m.len(), width as usize * height as usize);
    let mut presented = rgba8.to_vec();
    let width_denominator = width.saturating_sub(1).max(1) as f32;
    let height_denominator = height.saturating_sub(1).max(1) as f32;
    for (pixel_index, depth_m) in depth_m.iter().copied().enumerate() {
        let x = (pixel_index as u32 % width) as f32 / width_denominator;
        let y = (pixel_index as u32 / width) as f32 / height_denominator;
        let byte_index = pixel_index * 4;
        if !depth_m.is_finite() || depth_m >= far_m * 0.995 {
            let horizon_mix = smoothstep(0.02, 0.82, y);
            let top = [0.28_f32, 0.50, 0.72];
            let horizon = [0.76_f32, 0.84, 0.89];
            let sun_distance_sq = (x - 0.78).powi(2) + ((y - 0.20) * 1.35).powi(2);
            let sun_glow = (1.0 - sun_distance_sq / 0.055).clamp(0.0, 1.0).powi(3);
            for channel in 0..3 {
                let sky = top[channel] + (horizon[channel] - top[channel]) * horizon_mix;
                let sun = [1.0_f32, 0.91, 0.72][channel];
                presented[byte_index + channel] = to_srgb_byte(sky + (sun - sky) * sun_glow * 0.72);
            }
            presented[byte_index + 3] = 255;
            continue;
        }

        let fog = smoothstep(25.0, 92.0, depth_m) * 0.42;
        let atmospheric = [0.70_f32, 0.77, 0.80];
        let mut color = [0.0_f32; 3];
        for channel in 0..3 {
            let source = f32::from(rgba8[byte_index + channel]) / 255.0;
            color[channel] = source + (atmospheric[channel] - source) * fog;
        }
        let luma = color[0] * 0.2126 + color[1] * 0.7152 + color[2] * 0.0722;
        for channel in &mut color {
            *channel = luma + (*channel - luma) * 0.94;
            *channel = 0.5 + (*channel - 0.5) * 1.055;
        }
        color[0] *= 1.018;
        color[2] *= 0.985;
        let vignette_radius_sq = ((x - 0.5) * 1.50).powi(2) + ((y - 0.48) * 1.05).powi(2);
        let vignette = 1.0 - vignette_radius_sq.clamp(0.0, 1.0) * 0.095;
        for channel in 0..3 {
            presented[byte_index + channel] = to_srgb_byte(color[channel] * vignette);
        }
        presented[byte_index + 3] = 255;
    }
    presented
}

fn smoothstep(edge_min: f32, edge_max: f32, value: f32) -> f32 {
    let t = ((value - edge_min) / (edge_max - edge_min)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn to_srgb_byte(value: f32) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0).round() as u8
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=224:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg PLATEAU GIF encode failed"))
}

/// Colorizes a linear depth buffer with the same near-to-far ramp as the LiDAR bands.
///
/// Yellow is near, green is mid-range, and blue is far, so the depth inset reads
/// against the intensity-colored point cloud without a second legend.
fn depth_ramp_rgba(depth_m: &[f32], far_m: f32) -> Vec<u8> {
    const NEAR: [f32; 3] = [1.0, 0.86, 0.08];
    const MID: [f32; 3] = [0.04, 1.0, 0.52];
    const FAR: [f32; 3] = [0.08, 0.42, 1.0];
    let far_m = if far_m > 0.0 { far_m } else { 1.0 };

    let mut rgba = Vec::with_capacity(depth_m.len() * 4);
    for depth in depth_m {
        let t = if depth.is_finite() {
            (depth / far_m).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let (from, to, local) = if t < 0.5 {
            (NEAR, MID, t * 2.0)
        } else {
            (MID, FAR, (t - 0.5) * 2.0)
        };
        for channel in 0..3 {
            let value = from[channel] + (to[channel] - from[channel]) * local;
            rgba.push((value.clamp(0.0, 1.0) * 255.0).round() as u8);
        }
        rgba.push(255);
    }
    rgba
}

/// Mutable RGBA8 destination for inset compositing.
struct FrameBuffer<'a> {
    pixels: &'a mut [u8],
    width: u32,
    height: u32,
}

/// Read-only RGBA8 source for inset compositing.
#[derive(Clone, Copy)]
struct InsetImage<'a> {
    pixels: &'a [u8],
    width: u32,
    height: u32,
}

/// Copies an inset image into the frame at `origin`, drawing a border around it.
fn blit_inset(frame: &mut FrameBuffer<'_>, inset: InsetImage<'_>, origin: (u32, u32)) {
    const BORDER: [u8; 4] = [235, 240, 248, 255];
    let (origin_x, origin_y) = origin;
    let (frame_width, frame_height) = (frame.width, frame.height);
    let (inset_width, inset_height) = (inset.width, inset.height);
    let pixels = &mut *frame.pixels;

    let mut put = |x: i64, y: i64, pixel: [u8; 4]| {
        if x < 0 || y < 0 || x >= i64::from(frame_width) || y >= i64::from(frame_height) {
            return;
        }
        let offset = (y as usize * frame_width as usize + x as usize) * 4;
        if offset + 4 <= pixels.len() {
            pixels[offset..offset + 4].copy_from_slice(&pixel);
        }
    };

    for y in 0..inset_height {
        for x in 0..inset_width {
            let source = (y as usize * inset_width as usize + x as usize) * 4;
            if source + 4 > inset.pixels.len() {
                continue;
            }
            let pixel = [
                inset.pixels[source],
                inset.pixels[source + 1],
                inset.pixels[source + 2],
                inset.pixels[source + 3],
            ];
            put(i64::from(origin_x + x), i64::from(origin_y + y), pixel);
        }
    }

    for thickness in 0..CAMERA_INSET_BORDER_PX {
        let offset = i64::from(thickness) + 1;
        for x in -offset..i64::from(inset_width) + offset {
            put(
                i64::from(origin_x) + x,
                i64::from(origin_y) - offset,
                BORDER,
            );
            put(
                i64::from(origin_x) + x,
                i64::from(origin_y + inset_height) + offset - 1,
                BORDER,
            );
        }
        for y in -offset..i64::from(inset_height) + offset {
            put(
                i64::from(origin_x) - offset,
                i64::from(origin_y) + y,
                BORDER,
            );
            put(
                i64::from(origin_x + inset_width) + offset - 1,
                i64::from(origin_y) + y,
                BORDER,
            );
        }
    }
}

/// Draws the onboard RGB and depth camera views into the bottom-left of a frame.
fn blit_camera_insets(
    target: &mut FrameBuffer<'_>,
    color: InsetImage<'_>,
    depth_m: &[f32],
    far_m: f32,
) {
    let margin = CAMERA_INSET_MARGIN_PX;
    let inset_top = target.height.saturating_sub(margin + color.height);

    blit_inset(
        target,
        InsetImage {
            pixels: color.pixels,
            width: color.width,
            height: color.height,
        },
        (margin, inset_top),
    );

    let depth_rgba = depth_ramp_rgba(depth_m, far_m);
    blit_inset(
        target,
        InsetImage {
            pixels: &depth_rgba,
            width: color.width,
            height: color.height,
        },
        (margin * 2 + color.width, inset_top),
    );
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn official_sanjo_subset_imports_and_selects_station_road() {
        let source = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/sanjo_2025");
        let output =
            std::env::temp_dir().join(format!("rne-plateau-sanjo-test-{}", std::process::id()));
        if output.exists() {
            fs::remove_dir_all(&output).expect("remove stale Sanjo output");
        }
        let buildings = import_citygml_file(
            &source.join("56383756_bldg_6697.gml"),
            &output.join("buildings"),
            &ImportOptions {
                tile_name: "sanjo-buildings".into(),
                coordinate_mode: CoordinateMode::GeographicDegrees,
                origin: Some(SANJO_ORIGIN),
                ..ImportOptions::default()
            },
        )
        .expect("import official Sanjo buildings");
        let roads = import_citygml_file(
            &source.join("56383756_tran_6697.gml"),
            &output.join("roads"),
            &ImportOptions {
                tile_name: "sanjo-roads".into(),
                coordinate_mode: CoordinateMode::GeographicDegrees,
                origin: Some(SANJO_ORIGIN),
                ..ImportOptions::default()
            },
        )
        .expect("import official Sanjo roads");
        assert_eq!(buildings.building_count, 213);
        assert_eq!(buildings.lod2_building_count, 1);
        assert_eq!(buildings.textured_surface_count, 37);
        assert_eq!(roads.road_count, 59);
        assert_eq!(roads.lane_count, 84);
        let lanes = select_station_road_lanes(&roads.lanes);
        assert_eq!(lanes.len(), 2);
        assert!(lane_length_m(&lanes[0]) >= 45.0);
        assert!(lane_distance_to_station_m(&lanes[0]) < 30.0);
        let turn_lane = select_station_turn_lane(&roads.lanes, &lanes[0]);
        assert!(turn_lane
            .lane_id
            .starts_with("tran_b9e43a5f-9251-424a-9b40-5128df71a3b3/"));
        assert!(
            lane_direction(&turn_lane)
                .dot(lane_direction(&lanes[0]))
                .abs()
                < 0.50
        );
        let imported_traffic =
            load_traffic_asset(&roads.traffic_path).expect("load official traffic asset");
        let topology = build_traffic_topology(
            TrafficId::new("plateau:sanjo/test-topology").expect("test topology ID"),
            std::slice::from_ref(&imported_traffic.network),
            TopologyBuildConfig::default(),
        )
        .expect("build official topology");
        assert_eq!(topology.stats.lane_count, 84);
        assert_eq!(topology.stats.junction_count, 26);
        assert_eq!(topology.stats.connection_count, 137);
        assert_eq!(topology.stats.conflict_pair_count, 128);
        let scenario = build_city_traffic_scenario(&topology.network);
        let planned = select_city_lane_route(&topology.network);
        assert_eq!(scenario.lane_routes[0], planned);
        assert_eq!(planned.lane_ids.len(), 15);
        assert_eq!(lane_route_turn_counts(&topology.network, &planned), (6, 4));
        assert_eq!(scenario.routes.len(), CITY_ROUTE_COUNT);
        assert_eq!(scenario.actors.len(), CITY_ACTOR_COUNT);
        assert_eq!(
            scenario
                .actors
                .iter()
                .map(|actor| actor.route_id.clone())
                .collect::<BTreeSet<_>>()
                .len(),
            CITY_ROUTE_COUNT
        );
        assert_eq!(
            scenario
                .actors
                .iter()
                .map(|actor| actor.class)
                .collect::<BTreeSet<_>>()
                .len(),
            4
        );
        let runtime_route = scenario
            .routes
            .get(
                &TrafficId::new("plateau:sanjo/runtime-route-00").expect("representative route ID"),
            )
            .expect("representative route");
        let signal_distances_m = scenario
            .signals
            .iter()
            .filter(|signal| signal.route_id == *runtime_route.id())
            .map(|signal| signal.stop_distance_m)
            .collect::<Vec<_>>();
        let forward = replay_city_fleet(&topology.network, &scenario, false);
        let reverse = replay_city_fleet(&topology.network, &scenario, true);
        assert_eq!(forward, reverse);
        assert_eq!(forward.stable_hash, 12_942_443_943_866_480_899);
        assert_eq!(forward.signal_violation_count, 0);
        assert_eq!(forward.collision_count, 0);
        assert!(forward.minimum_gap_m >= 2.0 - 1.0e-9);
        assert!(forward.maximum_active_reservations > 0);
        assert!(forward.flow.average_speed_m_s > 0.0);
        assert!(forward.flow.cumulative_waiting_time_s > 0.0);

        let lidar_traffic = simulate_city_fleet_frames(&topology.network, &scenario, 12);
        let lidar_actor_index = (0..CITY_ACTOR_COUNT)
            .max_by(|left, right| {
                city_vehicle_motion_m(&lidar_traffic, *left)
                    .total_cmp(&city_vehicle_motion_m(&lidar_traffic, *right))
                    .then_with(|| right.cmp(left))
            })
            .expect("official Sanjo LiDAR camera actor");
        // The capture rides the dynamic-bicycle primary, exactly as the full run does.
        let dynamic_primary = apply_dynamic_primary(&lidar_traffic, lidar_actor_index);
        assert_eq!(
            dynamic_primary,
            apply_dynamic_primary(&lidar_traffic, lidar_actor_index),
            "dynamic primary must be deterministic"
        );
        assert!(dynamic_primary.maximum_deviation_m < DYNAMIC_PRIMARY_DEVIATION_LIMIT_M);
        // The swap touches exactly one actor; the other 99 rows are bit-identical.
        for (dynamic_vehicles, ghost_vehicles) in dynamic_primary.frames.iter().zip(&lidar_traffic)
        {
            for (index, (dynamic, ghost)) in dynamic_vehicles.iter().zip(ghost_vehicles).enumerate()
            {
                if index != lidar_actor_index {
                    assert_eq!(dynamic, ghost);
                }
            }
        }
        let lidar_traffic = dynamic_primary.frames;
        let mut lidar_building_bundle =
            load_scene_bundle(&buildings.scene_path).expect("load LiDAR building bundle");
        flatten_buildings_to_road_datum(&mut lidar_building_bundle);
        let lidar_road_bundle =
            load_scene_bundle(&roads.scene_path).expect("load LiDAR road bundle");
        let mut forward_lidar_world = World::new();
        spawn_scene_bundle(
            &mut forward_lidar_world,
            &lidar_building_bundle,
            None,
            SpawnSceneOptions::default(),
        )
        .expect("spawn forward LiDAR buildings");
        spawn_scene_bundle(
            &mut forward_lidar_world,
            &lidar_road_bundle,
            None,
            SpawnSceneOptions::default(),
        )
        .expect("spawn forward LiDAR roads");
        let mut reverse_lidar_world = World::new();
        spawn_scene_bundle(
            &mut reverse_lidar_world,
            &lidar_building_bundle,
            None,
            SpawnSceneOptions::default(),
        )
        .expect("spawn reverse LiDAR buildings");
        spawn_scene_bundle(
            &mut reverse_lidar_world,
            &lidar_road_bundle,
            None,
            SpawnSceneOptions::default(),
        )
        .expect("spawn reverse LiDAR roads");
        let forward_lidar = capture_city_lidar_frames(
            &mut forward_lidar_world,
            &lidar_traffic,
            lidar_actor_index,
            false,
        );
        let reverse_lidar = capture_city_lidar_frames(
            &mut reverse_lidar_world,
            &lidar_traffic,
            lidar_actor_index,
            true,
        );
        assert_eq!(forward_lidar.stable_hash, reverse_lidar.stable_hash);
        assert_eq!(forward_lidar.stable_hash, 13_248_311_255_248_989_536);
        assert_eq!(forward_lidar.total_returns, reverse_lidar.total_returns);
        assert_eq!(forward_lidar.multi_returns, reverse_lidar.multi_returns);
        assert_eq!(
            forward_lidar.saturated_returns,
            reverse_lidar.saturated_returns
        );
        assert!(forward_lidar.total_returns >= 12 * 24);
        assert!(forward_lidar.multi_returns > 0);
        // Retroreflective licence plates must drive the detector into saturation.
        assert!(forward_lidar.saturated_returns > 0);

        // The onboard camera rides the same actor and must be reproducible headlessly.
        let camera_scene = render_scene_from_world(&mut forward_lidar_world);
        let camera_assets = VehicleRenderAssets::load();
        let first_camera = capture_city_camera_frames(
            &camera_scene,
            &camera_assets,
            &lidar_traffic,
            lidar_actor_index,
        );
        let second_camera = capture_city_camera_frames(
            &camera_scene,
            &camera_assets,
            &lidar_traffic,
            lidar_actor_index,
        );
        assert_eq!(first_camera, second_camera);
        assert_eq!(first_camera.frames.len(), lidar_traffic.len());
        assert_eq!(
            first_camera.pixels_per_frame,
            (CAMERA_WIDTH * CAMERA_HEIGHT) as usize
        );
        assert!(first_camera.nearest_depth_m > 0.0);
        assert!(first_camera.nearest_depth_m.is_finite());
        assert!(first_camera.mean_center_depth_m > 0.0);
        // The view must change as the host vehicle drives.
        assert!(first_camera
            .frames
            .windows(2)
            .any(|pair| pair[0] != pair[1]));

        let probe = &forward_lidar.frames[1].cloud;
        assert!(probe.attributes_are_aligned());
        // Every elevation channel of the 3D scanner contributes returns.
        let rings = probe
            .channel_indices
            .iter()
            .copied()
            .collect::<BTreeSet<_>>();
        assert_eq!(rings.len(), usize::from(LIDAR_CHANNEL_COUNT));
        // Returns spread vertically instead of lying in one scan plane.
        let heights = probe.points_m.iter().map(|point| point.y);
        let lowest = heights.clone().fold(f64::INFINITY, f64::min);
        let highest = heights.fold(f64::NEG_INFINITY, f64::max);
        assert!(highest - lowest > 2.0);
        // Emission times sweep one revolution, which is what motion distortion needs.
        assert!(probe.timestamps_s.iter().all(|time| *time >= 0.0));
        assert!(probe.scan_duration_s() > 0.0);
        assert!(probe.scan_duration_s() < LIDAR_ROTATION_PERIOD_S);

        let mut debug_scene = RenderScene::new();
        append_traffic_debug_overlay(
            &mut debug_scene,
            &topology.network,
            &scenario.routes,
            runtime_route,
            &signal_distances_m,
            TrafficDebugOverlay {
                lanes: true,
                route: true,
                signals: true,
                connections: true,
                conflict_points: true,
            },
        );
        assert_eq!(debug_scene.items.len(), CITY_ROUTE_COUNT + 4);
        fs::remove_dir_all(output).expect("remove Sanjo output");
    }

    #[test]
    fn showcase_citygml_is_deterministic_and_importable() {
        let first = showcase_citygml();
        assert_eq!(first, showcase_citygml());
        let output =
            std::env::temp_dir().join(format!("rne-plateau-showcase-test-{}", std::process::id()));
        if output.exists() {
            fs::remove_dir_all(&output).expect("remove stale showcase output");
        }
        let source = output.join("source");
        let appearance = source.join("appearance");
        fs::create_dir_all(&appearance).expect("create showcase source");
        let citygml_path = source.join("synthetic_plateau_drive_showcase.gml");
        fs::write(&citygml_path, first).expect("write showcase CityGML");
        write_facade_texture(&appearance.join("facade.png"));
        let result = import_citygml_file(
            &citygml_path,
            &output.join("imported"),
            &ImportOptions {
                tile_name: "showcase-test".into(),
                coordinate_mode: CoordinateMode::ProjectedMeters,
                origin: Some(SourceOrigin {
                    first_deg_or_m: 0.0,
                    second_deg_or_m: 0.0,
                    height_m: 0.0,
                }),
                ..ImportOptions::default()
            },
        )
        .expect("import generated showcase");
        assert_eq!(result.building_count, 10);
        assert_eq!(result.lod2_building_count, 10);
        assert_eq!(result.textured_surface_count, 40);
        assert_eq!(result.road_count, 1);
        assert_eq!(result.lane_count, 2);
        assert_eq!(result.triangle_count, 122);
        fs::remove_dir_all(output).expect("remove showcase output");
    }

    #[test]
    fn simclock_traffic_is_deterministic_and_stays_in_derived_lanes() {
        assert_eq!(drone_position(0.5), drone_position(0.5));
        let lanes = vec![
            ImportedLane {
                lane_id: "road-main/surface-0000/lane-0".into(),
                road_source_id: "road-main".into(),
                centerline_m: [[-1.5, 0.05, -17.0], [-1.5, 0.05, 17.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisPositive,
            },
            ImportedLane {
                lane_id: "road-main/surface-0000/lane-1".into(),
                road_source_id: "road-main".into(),
                centerline_m: [[1.5, 0.05, 17.0], [1.5, 0.05, -17.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisNegative,
            },
        ];
        let first = simulate_two_way_traffic(&lanes, CAR_FRAME_COUNT);
        let second = simulate_two_way_traffic(&lanes, CAR_FRAME_COUNT);
        assert_eq!(first, second);
        assert!(first.0.last().unwrap().transform.translation.z > 12.0);
        assert!(first.0.iter().any(|frame| frame.braking));
        assert!(
            first.0.last().unwrap().wheel_rotation_rad
                > first.0.first().unwrap().wheel_rotation_rad
        );
        for (lane, frames) in [(&lanes[0], &first.0), (&lanes[1], &first.1)] {
            let lane_x = lane.centerline_m[0][0];
            for frame in frames {
                assert!((frame.transform.translation.x - lane_x).abs() < 0.05);
                assert!(frame.transform.translation.z >= -17.05);
                assert!(frame.transform.translation.z <= 17.05);
            }
        }
    }

    #[test]
    fn signalized_turn_stops_on_red_then_follows_outgoing_lane() {
        let lanes = vec![
            ImportedLane {
                lane_id: "incoming/surface-0000/lane-0".into(),
                road_source_id: "incoming".into(),
                centerline_m: [[0.0, 0.05, -40.0], [0.0, 0.05, 0.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisPositive,
            },
            ImportedLane {
                lane_id: "incoming/surface-0000/lane-1".into(),
                road_source_id: "incoming".into(),
                centerline_m: [[3.0, 0.05, 0.0], [3.0, 0.05, -40.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisNegative,
            },
        ];
        let outgoing = ImportedLane {
            lane_id: "outgoing/surface-0000/lane-0".into(),
            road_source_id: "outgoing".into(),
            centerline_m: [[2.0, 0.05, 2.0], [42.0, 0.05, 2.0]],
            width_m: 3.0,
            travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisPositive,
        };
        let first = simulate_signalized_turn(&lanes, &outgoing, CAR_FRAME_COUNT);
        let second = simulate_signalized_turn(&lanes, &outgoing, CAR_FRAME_COUNT);
        assert_eq!(first, second);
        assert_eq!(signal_phase_at(6.99), SignalPhase::Red);
        assert_eq!(signal_phase_at(7.0), SignalPhase::Green);
        let green_frame = (SIGNAL_GREEN_TIME_S * RENDER_HZ as f64) as usize;
        assert!(first.0[..green_frame]
            .iter()
            .any(|frame| frame.speed_m_s < 0.15));
        assert!(first.0[..green_frame].iter().any(|frame| frame.braking));
        assert!(first.0[..green_frame].iter().all(|frame| {
            (first.2.stop_point - frame.transform.translation).dot(first.2.incoming_direction)
                > -0.5
        }));
        let final_frame = first.0.last().expect("turn frame");
        assert!(
            (final_frame.transform.translation - first.2.intersection)
                .dot(first.2.outgoing_direction)
                > 8.0
        );
        let final_forward = final_frame.transform.rotation * Vec3::X;
        assert!(final_forward.dot(first.2.outgoing_direction) > 0.75);
    }

    #[test]
    fn kenney_vehicle_assets_load_with_independent_steered_wheels() {
        let assets = VehicleRenderAssets::load();
        assert!(
            assets
                .body_meshes
                .iter()
                .map(|mesh| mesh.triangle_count())
                .sum::<usize>()
                > 100
        );
        assert!(
            assets
                .wheel_meshes
                .iter()
                .map(|mesh| mesh.triangle_count())
                .sum::<usize>()
                > 50
        );
        assert_ne!(
            assets.red_body_texture.hash_pixels(),
            assets.blue_body_texture.hash_pixels()
        );

        let mut scene = RenderScene::new();
        let vehicle = VehicleFrame {
            transform: Transform3::from_translation_rotation(
                Vec3::new(0.0, 0.65, 0.0),
                Quat::IDENTITY,
            ),
            speed_m_s: 4.0,
            steering_rad: 0.25,
            wheel_rotation_rad: 1.5,
            braking: true,
            class: CityVehicleClass::Sedan,
        };
        append_car(&mut scene, &assets, vehicle, &assets.red_body_texture);

        let mesh_count = assets.body_meshes.len() + 4 * assets.wheel_meshes.len();
        assert_eq!(scene.items.len(), mesh_count + 3);
        assert!(scene.items[..mesh_count]
            .iter()
            .all(|item| item.mesh.is_some() && item.base_color_texture.is_some()));
        let rear_wheel = &scene.items[assets.body_meshes.len()].transform;
        let front_wheel =
            &scene.items[assets.body_meshes.len() + 2 * assets.wheel_meshes.len()].transform;
        assert_ne!(rear_wheel.rotation, front_wheel.rotation);
        assert!(scene.items[mesh_count..mesh_count + 2]
            .iter()
            .all(|item| item.color_rgba[0] == 1.0));
        assert_eq!(
            scene.items[mesh_count + 2].color_rgba,
            [0.78, 0.80, 0.78, 1.0]
        );
    }

    #[test]
    fn cinematic_streetscape_reserves_capacity_for_dynamic_actors() {
        let mut scene = RenderScene::new();
        let lanes = vec![
            ImportedLane {
                lane_id: "road-main/surface-0000/lane-0".into(),
                road_source_id: "road-main".into(),
                centerline_m: [[-1.5, 0.05, -44.0], [-1.5, 0.05, 44.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisPositive,
            },
            ImportedLane {
                lane_id: "road-main/surface-0000/lane-1".into(),
                road_source_id: "road-main".into(),
                centerline_m: [[1.5, 0.05, 44.0], [1.5, 0.05, -44.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisNegative,
            },
        ];
        let blocking_building = Footprint {
            min_x_m: 5.0,
            max_x_m: 8.0,
            min_z_m: -29.0,
            max_z_m: -23.0,
        };
        let first =
            append_city_streetscape(&mut scene, &lanes, std::slice::from_ref(&blocking_building));
        let mut repeated_scene = RenderScene::new();
        let second = append_city_streetscape(
            &mut repeated_scene,
            &lanes,
            std::slice::from_ref(&blocking_building),
        );

        assert!(scene.items.len() <= MAX_STATIC_SCENE_ITEMS);
        assert!(scene.items.len() > 50);
        assert_eq!(first, second);
        assert!(first.iter().all(|fixture| {
            !blocking_building.overlaps_disc(fixture.center, fixture.clearance_radius_m)
        }));
        assert!(
            first.len() < 18,
            "the building should reject at least one candidate fixture"
        );
    }

    #[test]
    fn showcase_round_primitives_apply_requested_dimensions() {
        let mut scene = RenderScene::new();
        push_sphere(&mut scene, Vec3::ZERO, 0.10, [1.0; 4]);
        push_cylinder(&mut scene, Vec3::ZERO, Quat::IDENTITY, 0.09, 5.2, [1.0; 4]);
        assert_eq!(scene.items[0].transform.scale, Vec3::splat(0.20));
        assert_eq!(scene.items[1].transform.scale, Vec3::new(0.18, 0.18, 5.2));
    }

    #[test]
    fn onboard_camera_looks_along_the_vehicle_heading() {
        for yaw_rad in [0.0_f64, 0.7, -1.9, 3.0, 2.4] {
            let vehicle = VehicleFrame {
                transform: Transform3::from_translation_rotation(
                    Vec3::new(3.0, 0.0, -7.0),
                    Quat::from_rotation_y(yaw_rad),
                ),
                speed_m_s: 4.0,
                steering_rad: 0.0,
                wheel_rotation_rad: 0.0,
                braking: false,
                class: CityVehicleClass::Sedan,
            };
            let pose = vehicle_camera_transform(vehicle);
            let heading = vehicle.transform.rotation * Vec3::X;
            // The render view convention looks along local -Z.
            let view_forward = pose.rotation * Vec3::NEG_Z;

            let horizontal = Vec3::new(view_forward.x, 0.0, view_forward.z).normalize_or_zero();
            assert!(
                horizontal.dot(heading) > 0.999,
                "camera must face the vehicle heading at yaw {yaw_rad}"
            );
            // A forward ADAS camera is tilted slightly down.
            assert!(view_forward.y < 0.0);
            // It is mounted above the road and ahead of the vehicle origin.
            assert!(pose.translation.y > 0.5);
            assert!((pose.translation - vehicle.transform.translation).dot(heading) > 0.0);
        }
    }

    #[test]
    fn depth_ramp_runs_from_near_yellow_to_far_blue() {
        let ramp = depth_ramp_rgba(&[0.0, 50.0, 100.0, f32::INFINITY], 100.0);
        assert_eq!(ramp.len(), 16);

        // Near is yellow, mid is green, far is blue.
        assert_eq!(&ramp[0..4], &[255, 219, 20, 255]);
        assert_eq!(&ramp[4..8], &[10, 255, 133, 255]);
        assert_eq!(&ramp[8..12], &[20, 107, 255, 255]);
        // Non-finite depth is treated as the far plane rather than producing garbage.
        assert_eq!(&ramp[8..12], &ramp[12..16]);
    }

    #[test]
    fn camera_insets_stay_inside_the_frame_and_leave_the_action_visible() {
        let frame_width = 1_280_u32;
        let frame_height = 720_u32;
        let mut frame = vec![0_u8; (frame_width * frame_height * 4) as usize];
        let color_rgba8 = (0..CAMERA_WIDTH * CAMERA_HEIGHT)
            .flat_map(|_| [9_u8, 9, 9, 255])
            .collect::<Vec<_>>();
        let depth_m = vec![25.0_f32; (CAMERA_WIDTH * CAMERA_HEIGHT) as usize];

        blit_camera_insets(
            &mut FrameBuffer {
                pixels: &mut frame,
                width: frame_width,
                height: frame_height,
            },
            InsetImage {
                pixels: &color_rgba8,
                width: CAMERA_WIDTH,
                height: CAMERA_HEIGHT,
            },
            &depth_m,
            80.0,
        );

        let pixel = |x: u32, y: u32| {
            let offset = (y as usize * frame_width as usize + x as usize) * 4;
            [
                frame[offset],
                frame[offset + 1],
                frame[offset + 2],
                frame[offset + 3],
            ]
        };
        let inset_top = frame_height - CAMERA_INSET_MARGIN_PX - CAMERA_HEIGHT;

        // The RGB inset is copied verbatim.
        assert_eq!(
            pixel(CAMERA_INSET_MARGIN_PX + 1, inset_top + 1),
            [9, 9, 9, 255]
        );
        // The depth inset sits beside it and is colorized, not copied.
        let depth_x = CAMERA_INSET_MARGIN_PX * 2 + CAMERA_WIDTH + 1;
        assert_ne!(pixel(depth_x, inset_top + 1), [9, 9, 9, 255]);
        assert_eq!(pixel(depth_x, inset_top + 1)[3], 255);
        // Both insets are bordered.
        assert_eq!(
            pixel(CAMERA_INSET_MARGIN_PX, inset_top - 1),
            [235, 240, 248, 255]
        );
        // The upper two thirds of the frame stay untouched by the overlay.
        assert_eq!(pixel(frame_width / 2, frame_height / 3), [0, 0, 0, 0]);
        // Nothing wrote past the right edge of the second inset.
        assert!(depth_x + CAMERA_WIDTH + CAMERA_INSET_BORDER_PX < frame_width);
    }

    #[test]
    fn cinematic_postprocess_is_deterministic_and_depth_aware() {
        let rgba8 = vec![
            80, 110, 140, 255, 80, 110, 140, 255, 60, 70, 75, 255, 60, 70, 75, 255,
        ];
        let depth_m = vec![140.0, 140.0, 8.0, 90.0];
        let first = cinematic_postprocess(&rgba8, &depth_m, 2, 2, 140.0);
        let second = cinematic_postprocess(&rgba8, &depth_m, 2, 2, 140.0);
        assert_eq!(first, second);
        assert_ne!(first, rgba8);
        assert_ne!(&first[0..3], &rgba8[0..3]);
        assert!(
            first[14] > first[10],
            "distant geometry should receive blue atmospheric haze"
        );
        assert!(first.chunks_exact(4).all(|pixel| pixel[3] == 255));
    }
}
