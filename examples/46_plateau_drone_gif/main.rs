//! Imports a synthetic PLATEAU tile and renders drone and car traversal GIFs.

use png::{BitDepth, ColorType, Encoder};
use rne_assets::{load_scene_bundle, mesh_package_roots, spawn_scene_bundle, SpawnSceneOptions};
use rne_core::{SimClock, SimDuration};
use rne_ecs::{spawn_named, Name, World};
use rne_math::{Hertz, Quat, Transform3 as MathTransform3, Vec3};
use rne_plateau::{import_citygml_str, CoordinateMode, ImportOptions, ImportedLane, SourceOrigin};
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, Visual, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{
    ackermann_kinematics, command_ackermann_drive, pure_pursuit_steering, AckermannDrive,
};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 720;
const HEIGHT: u32 = 405;
const DRONE_FRAME_COUNT: usize = 48;
const CAR_FRAME_COUNT: usize = 96;
const RENDER_HZ: usize = 12;
const SIM_HZ: usize = 60;
const SIM_STEPS_PER_FRAME: usize = SIM_HZ / RENDER_HZ;
const CLEAR_COLOR: [f32; 4] = [0.16, 0.28, 0.42, 1.0];

#[derive(Clone, Copy, Debug, PartialEq)]
struct VehicleFrame {
    transform: Transform3,
    speed_m_s: f64,
    steering_rad: f64,
    wheel_rotation_rad: f64,
}

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
    let generated_dir = repo_root.join("target/plateau-city-drive-demo");
    let citygml = showcase_citygml();
    let result = import_citygml_str(
        &citygml,
        "synthetic_plateau_drive_showcase.gml",
        &generated_dir,
        &ImportOptions {
            tile_name: "plateau-city-drive".into(),
            coordinate_mode: CoordinateMode::ProjectedMeters,
            origin: Some(SourceOrigin {
                first_deg_or_m: 0.0,
                second_deg_or_m: 0.0,
                height_m: 0.0,
            }),
            world_seed: 46,
            ..ImportOptions::default()
        },
    )
    .expect("import synthetic PLATEAU tile");
    let bundle = load_scene_bundle(&result.scene_path).expect("load generated PLATEAU scene");
    let mut world = World::new();
    spawn_scene_bundle(&mut world, &bundle, None, SpawnSceneOptions::default())
        .expect("spawn generated PLATEAU scene headlessly");
    assert_eq!(result.building_count, 10);
    assert_eq!(result.road_count, 1);
    assert_eq!(result.lane_count, 2);
    assert_eq!(bundle.scene.objects.len(), 11);
    let (primary_traffic, opposing_traffic) =
        simulate_two_way_traffic(&result.lanes, CAR_FRAME_COUNT);
    println!(
        "PLATEAU tile ready: buildings={} roads={} lanes={} triangles={} scene={}",
        result.building_count,
        result.road_count,
        result.lane_count,
        result.triangle_count,
        result.scene_path.display()
    );

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless PLATEAU import completed");
        return;
    }
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable after successful headless import: {error}");
            return;
        }
    };
    let media_dir = repo_root.join("docs/media");
    let frames_dir = generated_dir.join("frames");
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old PLATEAU frames");
    }
    fs::create_dir_all(&frames_dir).expect("create PLATEAU frames");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut city_scene = render_scene_from_world(&mut world);
    let mesh_roots = mesh_package_roots(&bundle);
    let root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    city_scene
        .resolve_mesh_assets_with_roots(&root_refs)
        .expect("resolve generated PLATEAU meshes");
    append_city_streetscape(&mut city_scene);

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 8.5, 0.0),
        yaw_rad: -0.80,
        pitch_rad: 0.91,
        distance_m: 42.0,
    };
    for frame in 0..DRONE_FRAME_COUNT {
        let progress = frame as f64 / (DRONE_FRAME_COUNT - 1) as f64;
        let drone_position = drone_position(progress);
        let traffic_index = frame * (CAR_FRAME_COUNT - 1) / (DRONE_FRAME_COUNT - 1);
        let mut scene = city_scene.clone();
        append_flight_path(&mut scene, progress);
        append_traffic(
            &mut scene,
            primary_traffic[traffic_index],
            opposing_traffic[traffic_index],
        );
        append_drone(&mut scene, drone_position, progress);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render PLATEAU drone frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write PLATEAU drone frame");
    }

    let gif_path = media_dir.join("plateau-drone.gif");
    build_gif(&frames_dir, &gif_path).expect("encode PLATEAU drone GIF");
    image::open(frames_dir.join("frame-040.png"))
        .expect("read PLATEAU poster frame")
        .save(media_dir.join("plateau-drone.png"))
        .expect("write PLATEAU poster");
    fs::remove_dir_all(&frames_dir).expect("remove PLATEAU frame directory");

    fs::create_dir_all(&frames_dir).expect("create PLATEAU car frames");
    for frame in 0..CAR_FRAME_COUNT {
        let primary = primary_traffic[frame];
        let mut scene = city_scene.clone();
        append_traffic(&mut scene, primary, opposing_traffic[frame]);
        let car_camera = follow_camera(primary);
        let output = backend
            .render_scene_camera(&camera, &car_camera.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render PLATEAU car frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write PLATEAU car frame");
    }
    let car_gif_path = media_dir.join("plateau-car.gif");
    build_gif(&frames_dir, &car_gif_path).expect("encode PLATEAU car GIF");
    image::open(frames_dir.join("frame-032.png"))
        .expect("read PLATEAU car poster frame")
        .save(media_dir.join("plateau-car.png"))
        .expect("write PLATEAU car poster");
    fs::remove_dir_all(&frames_dir).expect("remove PLATEAU car frame directory");
    println!(
        "rendered PLATEAU drone and car media to {} and {}",
        gif_path.display(),
        car_gif_path.display()
    );
}

fn showcase_citygml() -> String {
    let mut xml = String::from(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!-- Synthetic CC0 PLATEAU-style showcase; contains no surveyed geometry. -->
<core:CityModel
    xmlns:core="http://www.opengis.net/citygml/2.0"
    xmlns:gml="http://www.opengis.net/gml"
    xmlns:bldg="http://www.opengis.net/citygml/building/2.0"
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
    xml.push_str("</core:CityModel>\n");
    xml
}

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
        "  <core:cityObjectMember>\n    <bldg:Building gml:id=\"{}\">\n      <gml:name>{}</gml:name>\n      <bldg:function>401</bldg:function>\n      <bldg:measuredHeight uom=\"m\">{h}</bldg:measuredHeight>\n      <bldg:lod1Solid><gml:Solid><gml:exterior><gml:CompositeSurface>\n",
        building.id, building.name
    ));
    for ring in rings {
        xml.push_str(&format!(
            "        <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension=\"3\">{ring}</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>\n"
        ));
    }
    xml.push_str(
        "      </gml:CompositeSurface></gml:exterior></gml:Solid></bldg:lod1Solid>\n    </bldg:Building>\n  </core:cityObjectMember>\n",
    );
}

fn render_scene_from_world(world: &mut World) -> RenderScene {
    let mut scene = RenderScene::new();
    let mut query = world.query::<(&Transform3, &Visual, Option<&Name>)>();
    for (transform, visual, name) in query.iter(world) {
        let color_rgba = name
            .filter(|name| name.0.starts_with("plateau_building_"))
            .map(|name| building_color(&name.0))
            .unwrap_or(visual.color_rgba);
        scene.items.push(RenderScene::item_from_visual(
            *transform,
            visual.shape.clone(),
            color_rgba,
            visual.local_offset,
        ));
    }
    scene
}

fn building_color(name: &str) -> [f32; 4] {
    const PALETTE: [[f32; 4]; 6] = [
        [0.64, 0.68, 0.71, 1.0],
        [0.72, 0.66, 0.58, 1.0],
        [0.58, 0.64, 0.69, 1.0],
        [0.69, 0.70, 0.66, 1.0],
        [0.62, 0.58, 0.55, 1.0],
        [0.73, 0.72, 0.68, 1.0],
    ];
    let hash = name.bytes().fold(2_166_136_261_u32, |hash, byte| {
        (hash ^ u32::from(byte)).wrapping_mul(16_777_619)
    });
    PALETTE[hash as usize % PALETTE.len()]
}

fn append_city_streetscape(scene: &mut RenderScene) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation: Vec3::new(0.0, -0.08, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(60.0, 0.12, 100.0),
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: [0.16, 0.20, 0.18, 1.0],
        mesh: None,
    });
    for side in [-1.0, 1.0] {
        push_box(
            scene,
            Vec3::new(side * 5.25, 0.06, 0.0),
            Quat::IDENTITY,
            Vec3::new(2.35, 0.14, 92.0),
            [0.43, 0.45, 0.44, 1.0],
        );
        push_box(
            scene,
            Vec3::new(side * 4.08, 0.11, 0.0),
            Quat::IDENTITY,
            Vec3::new(0.18, 0.24, 92.0),
            [0.68, 0.68, 0.64, 1.0],
        );
        push_box(
            scene,
            Vec3::new(side * 3.72, 0.045, 0.0),
            Quat::IDENTITY,
            Vec3::new(0.12, 0.045, 88.0),
            [0.86, 0.82, 0.58, 1.0],
        );
        for z in [-36.0, -18.0, 18.0, 36.0] {
            append_streetlight(scene, Vec3::new(side * 5.55, 0.0, z), -side);
        }
        for (index, z) in [-29.0, -11.0, 10.0, 29.0].into_iter().enumerate() {
            append_tree(
                scene,
                Vec3::new(side * (6.15 + index as f64 * 0.08), 0.0, z),
            );
        }
    }
    for segment in -10..=10 {
        push_box(
            scene,
            Vec3::new(0.0, 0.05, segment as f64 * 4.0),
            Quat::IDENTITY,
            Vec3::new(0.10, 0.045, 2.1),
            [0.88, 0.88, 0.84, 1.0],
        );
    }
    for z in [-1.35, -0.45, 0.45, 1.35] {
        push_box(
            scene,
            Vec3::new(0.0, 0.055, z),
            Quat::IDENTITY,
            Vec3::new(7.1, 0.05, 0.38),
            [0.90, 0.90, 0.86, 1.0],
        );
    }
    append_showcase_facades(scene);
}

fn append_streetlight(scene: &mut RenderScene, base: Vec3, road_direction: f64) {
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
        base + Vec3::new(road_direction * 0.55, 5.12, 0.0),
        Quat::IDENTITY,
        Vec3::new(1.1, 0.08, 0.08),
        [0.18, 0.20, 0.21, 1.0],
    );
    push_box(
        scene,
        base + Vec3::new(road_direction * 1.05, 5.02, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.48, 0.13, 0.24),
        [0.28, 0.30, 0.30, 1.0],
    );
    push_box(
        scene,
        base + Vec3::new(road_direction * 1.05, 4.94, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.34, 0.04, 0.16),
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
    push_sphere(
        scene,
        base + Vec3::new(0.0, 3.25, 0.0),
        1.25,
        [0.12, 0.34, 0.16, 1.0],
    );
}

fn append_showcase_facades(scene: &mut RenderScene) {
    for building in SHOWCASE_BUILDINGS {
        let road_face_x = if building.x_max_m < 0.0 {
            building.x_max_m + 0.025
        } else {
            building.x_min_m - 0.025
        };
        let mut floor_y_m = 2.2;
        while floor_y_m < building.height_m - 1.0 {
            push_box(
                scene,
                Vec3::new(
                    road_face_x,
                    floor_y_m,
                    (building.z_min_m + building.z_max_m) * 0.5,
                ),
                Quat::IDENTITY,
                Vec3::new(0.06, 1.05, building.z_max_m - building.z_min_m - 2.4),
                [0.07, 0.15, 0.21, 1.0],
            );
            floor_y_m += 2.8;
        }
        push_box(
            scene,
            Vec3::new(
                road_face_x,
                1.15,
                (building.z_min_m + building.z_max_m) * 0.5,
            ),
            Quat::IDENTITY,
            Vec3::new(0.08, 2.25, 1.45),
            [0.10, 0.12, 0.13, 1.0],
        );
    }
}

fn drone_position(progress: f64) -> Vec3 {
    let x = -20.0 + 40.0 * progress;
    let z = 8.5 - 17.0 * progress;
    let y = 14.5 + (progress * std::f64::consts::TAU).sin();
    Vec3::new(x, y, z)
}

fn append_flight_path(scene: &mut RenderScene, progress: f64) {
    let visible_markers = (progress * 20.0).floor() as usize;
    for marker in 0..=visible_markers {
        let marker_progress = marker as f64 / 20.0;
        let position = drone_position(marker_progress) - Vec3::new(0.0, 0.8, 0.0);
        push_box(
            scene,
            position,
            Quat::IDENTITY,
            Vec3::splat(0.24),
            [0.10, 0.70, 0.88, 0.75],
        );
    }
}

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
    let eye_direction = (-forward + right * 0.14).normalize_or_zero();
    CameraOrbit {
        focus: vehicle.transform.translation + forward * 4.0 + Vec3::new(0.0, 0.55, 0.0),
        yaw_rad: eye_direction.x.atan2(eye_direction.z),
        pitch_rad: 1.28,
        distance_m: 18.0,
    }
}

fn append_traffic(scene: &mut RenderScene, primary: VehicleFrame, opposing: VehicleFrame) {
    append_car(scene, primary, [0.84, 0.12, 0.045, 1.0]);
    append_car(scene, opposing, [0.045, 0.24, 0.72, 1.0]);
}

fn append_car(scene: &mut RenderScene, vehicle: VehicleFrame, color_rgba: [f32; 4]) {
    let center = vehicle.transform.translation;
    let rotation = vehicle.transform.rotation;
    push_box(
        scene,
        center,
        rotation,
        Vec3::new(4.35, 0.52, 1.82),
        color_rgba,
    );
    push_box(
        scene,
        center + rotation * Vec3::new(-0.15, 0.48, 0.0),
        rotation,
        Vec3::new(1.95, 0.65, 1.58),
        [0.12, 0.20, 0.27, 1.0],
    );
    push_box(
        scene,
        center + rotation * Vec3::new(0.82, 0.50, 0.0),
        rotation * Quat::from_rotation_z(-0.68),
        Vec3::new(0.08, 0.78, 1.50),
        [0.16, 0.26, 0.34, 1.0],
    );
    push_box(
        scene,
        center + rotation * Vec3::new(-1.12, 0.48, 0.0),
        rotation * Quat::from_rotation_z(0.64),
        Vec3::new(0.08, 0.72, 1.48),
        [0.14, 0.23, 0.30, 1.0],
    );
    for (x, color) in [
        (2.20, [0.98, 0.86, 0.42, 1.0]),
        (-2.20, [0.90, 0.04, 0.025, 1.0]),
    ] {
        for z in [-0.58, 0.58] {
            push_box(
                scene,
                center + rotation * Vec3::new(x, -0.02, z),
                rotation,
                Vec3::new(0.08, 0.18, 0.34),
                color,
            );
        }
    }
    for (x, z, steerable) in [
        (-1.34, -0.96, false),
        (-1.34, 0.96, false),
        (1.34, -0.96, true),
        (1.34, 0.96, true),
    ] {
        let wheel_rotation = rotation
            * Quat::from_rotation_y(if steerable { vehicle.steering_rad } else { 0.0 })
            * Quat::from_rotation_z(vehicle.wheel_rotation_rad);
        push_cylinder(
            scene,
            center + rotation * Vec3::new(x, -0.32, z),
            wheel_rotation,
            0.36,
            0.24,
            [0.012, 0.016, 0.020, 1.0],
        );
        push_box(
            scene,
            center + rotation * Vec3::new(x, -0.32, z),
            wheel_rotation,
            Vec3::new(0.58, 0.07, 0.26),
            [0.58, 0.61, 0.64, 1.0],
        );
    }
}

fn append_drone(scene: &mut RenderScene, center: Vec3, progress: f64) {
    let yaw = -0.4 + progress * 0.8;
    push_box(
        scene,
        center,
        Quat::from_rotation_y(yaw),
        Vec3::new(2.8, 0.70, 1.9),
        [0.09, 0.16, 0.22, 1.0],
    );
    for diagonal in [-1.0, 1.0] {
        push_box(
            scene,
            center + Vec3::new(0.0, 0.05, 0.0),
            Quat::from_rotation_y(yaw + diagonal * std::f64::consts::FRAC_PI_4),
            Vec3::new(5.6, 0.20, 0.20),
            [0.18, 0.26, 0.32, 1.0],
        );
    }
    let rotor_spin = progress * std::f64::consts::TAU * 10.0;
    for (x, z) in [(-2.0, -1.35), (-2.0, 1.35), (2.0, -1.35), (2.0, 1.35)] {
        let local = Quat::from_rotation_y(yaw) * Vec3::new(x, 0.18, z);
        push_box(
            scene,
            center + local,
            Quat::from_rotation_y(rotor_spin),
            Vec3::new(2.1, 0.08, 0.15),
            [0.20, 0.85, 0.95, 1.0],
        );
    }
    push_box(
        scene,
        center + Vec3::new(0.0, -0.42, 0.45),
        Quat::IDENTITY,
        Vec3::new(0.75, 0.55, 0.70),
        [0.92, 0.38, 0.12, 1.0],
    );
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
            scale: Vec3::ONE,
        },
        shape: VisualShape::Cylinder { radius_m, length_m },
        color_rgba,
        mesh: None,
    });
}

fn push_sphere(scene: &mut RenderScene, translation: Vec3, radius_m: f64, color_rgba: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::Sphere { radius_m },
        color_rgba,
        mesh: None,
    });
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
            "fps=12,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg PLATEAU GIF encode failed"))
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
    fn showcase_citygml_is_deterministic_and_importable() {
        let first = showcase_citygml();
        assert_eq!(first, showcase_citygml());
        let output =
            std::env::temp_dir().join(format!("rne-plateau-showcase-test-{}", std::process::id()));
        if output.exists() {
            fs::remove_dir_all(&output).expect("remove stale showcase output");
        }
        let result = import_citygml_str(
            &first,
            "synthetic_plateau_drive_showcase.gml",
            &output,
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
        for (lane, frames) in [(&lanes[0], &first.0), (&lanes[1], &first.1)] {
            let lane_x = lane.centerline_m[0][0];
            for frame in frames {
                assert!((frame.transform.translation.x - lane_x).abs() < 0.05);
                assert!(frame.transform.translation.z >= -17.05);
                assert!(frame.transform.translation.z <= 17.05);
            }
        }
    }
}
