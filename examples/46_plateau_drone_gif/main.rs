//! Imports official PLATEAU data for Kita-Sanjo Station and renders a car traversal GIF.

use png::{BitDepth, ColorType, Encoder};
use rne_assets::{
    load_scene_bundle, mesh_package_roots, spawn_scene_bundle, SceneAssetBundle,
    SceneCollisionAsset, SpawnSceneOptions,
};
use rne_core::{SimClock, SimDuration};
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Transform3 as MathTransform3, Vec3};
use rne_plateau::{import_citygml_file, CoordinateMode, ImportOptions, ImportedLane, SourceOrigin};
use rne_render::{
    load_mesh_parts, Camera, ImageFrame, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh,
    Visual, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{
    ackermann_kinematics, command_ackermann_drive, pure_pursuit_steering, AckermannDrive,
};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const WIDTH: u32 = 1_280;
const HEIGHT: u32 = 720;
const CAR_FRAME_COUNT: usize = 96;
const RENDER_HZ: usize = 12;
const SIM_HZ: usize = 60;
const SIM_STEPS_PER_FRAME: usize = SIM_HZ / RENDER_HZ;
const CLEAR_COLOR: [f32; 4] = [0.34, 0.52, 0.70, 1.0];
const MAX_STATIC_SCENE_ITEMS: usize = 400;
const SANJO_ORIGIN: SourceOrigin = SourceOrigin {
    first_deg_or_m: 37.631_938_029_139_7,
    second_deg_or_m: 138.955_122_347_658_72,
    height_m: 0.0,
};
const KITA_SANJO_STATION_XZ_M: [f64; 2] = [59.46, -77.17];

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
    let showcase_lanes = select_station_road_lanes(&roads.lanes);
    let (primary_traffic, opposing_traffic) =
        simulate_two_way_traffic(&showcase_lanes, CAR_FRAME_COUNT);
    println!(
        "official PLATEAU tile ready: buildings={} lod2={} textured_surfaces={} roads={} lanes={} triangles={}",
        buildings.building_count,
        buildings.lod2_building_count,
        buildings.textured_surface_count,
        roads.road_count,
        roads.lane_count,
        buildings.triangle_count + roads.triangle_count,
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
    let mut mesh_roots = mesh_package_roots(&building_bundle);
    mesh_roots.extend(mesh_package_roots(&road_bundle));
    let root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    city_scene
        .resolve_mesh_assets_with_roots(&root_refs)
        .expect("resolve generated PLATEAU meshes");
    let building_footprints = building_footprints(&building_bundle);
    let street_fixtures =
        append_city_streetscape(&mut city_scene, &showcase_lanes, &building_footprints);
    append_lane_markings(&mut city_scene, &showcase_lanes);
    let vehicle_assets = VehicleRenderAssets::load();

    assert!(
        city_scene.items.len() <= MAX_STATIC_SCENE_ITEMS,
        "PLATEAU scene leaves insufficient room for moving actors"
    );
    println!(
        "streetscape ready: fixtures={} static_scene_items={}",
        street_fixtures.len(),
        city_scene.items.len()
    );
    let mut camera = Camera::new(WIDTH, HEIGHT, 0.86);
    camera.far_m = 280.0;
    for frame in 0..CAR_FRAME_COUNT {
        let primary = primary_traffic[frame];
        let mut scene = city_scene.clone();
        append_traffic(
            &mut scene,
            &vehicle_assets,
            primary,
            opposing_traffic[frame],
        );
        let car_camera = follow_camera(primary);
        let output = backend
            .render_scene_camera(&camera, &car_camera.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render PLATEAU car frame");
        let presented = cinematic_postprocess(
            &output.color.rgba8,
            &output.depth.depth_m,
            output.color.width,
            output.color.height,
            camera.far_m as f32,
        );
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &presented,
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
        "rendered official PLATEAU car media to {}",
        car_gif_path.display()
    );
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

fn lane_length_m(lane: &ImportedLane) -> f64 {
    let start = Vec3::from_array(lane.centerline_m[0]);
    let end = Vec3::from_array(lane.centerline_m[1]);
    (end - start).length()
}

fn lane_distance_to_station_m(lane: &ImportedLane) -> f64 {
    let start = Vec3::from_array(lane.centerline_m[0]);
    let end = Vec3::from_array(lane.centerline_m[1]);
    let station = Vec3::new(KITA_SANJO_STATION_XZ_M[0], 0.05, KITA_SANJO_STATION_XZ_M[1]);
    let segment = end - start;
    let progress = ((station - start).dot(segment) / segment.length_squared()).clamp(0.0, 1.0);
    (station - (start + segment * progress)).length()
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
            braking: drive.speed_m_s > 0.2 && drive.target_speed_m_s + 0.05 < drive.speed_m_s,
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
        focus: vehicle.transform.translation + forward * 6.6 + Vec3::new(0.0, 0.42, 0.0),
        yaw_rad: eye_direction.x.atan2(eye_direction.z),
        pitch_rad: 1.40,
        distance_m: 15.5,
    }
}

fn append_traffic(
    scene: &mut RenderScene,
    assets: &VehicleRenderAssets,
    primary: VehicleFrame,
    opposing: VehicleFrame,
) {
    append_car(scene, assets, primary, &assets.red_body_texture);
    append_car(scene, assets, opposing, &assets.blue_body_texture);
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
            "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=224:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
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
