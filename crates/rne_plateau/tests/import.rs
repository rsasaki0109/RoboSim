use rne_assets::{load_scene_bundle, spawn_scene_bundle, SpawnSceneOptions};
use rne_ecs::World;
use rne_physics::Collider;
use rne_plateau::{import_citygml_file, CoordinateMode, ImportError, ImportOptions};
use rne_render::Visual;
use rne_traffic::{load_traffic_asset, AccuracyClass, AuthorityClass, LaneKind};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};

fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plateau_lod1_minimal.gml")
}

fn temp_output(label: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rne-plateau-{label}-{}", std::process::id()))
}

fn reset_dir(path: &Path) {
    if path.exists() {
        fs::remove_dir_all(path).expect("remove stale test output");
    }
    fs::create_dir_all(path).expect("create test output");
}

#[test]
fn imports_lod1_scene_with_stable_metadata_and_headless_colliders() {
    let output = temp_output("headless");
    reset_dir(&output);
    let result = import_citygml_file(
        &fixture_path(),
        &output,
        &ImportOptions {
            tile_name: "synthetic-city".into(),
            ..ImportOptions::default()
        },
    )
    .expect("import synthetic PLATEAU fixture");

    assert_eq!(result.building_count, 2);
    assert_eq!(result.lod2_building_count, 0);
    assert_eq!(result.textured_surface_count, 0);
    assert_eq!(result.road_count, 1);
    assert_eq!(result.lane_count, 2);
    assert_eq!(result.triangle_count, 26);
    assert_eq!(result.coordinate_mode, CoordinateMode::GeographicDegrees);

    let metadata: Value =
        serde_json::from_slice(&fs::read(&result.metadata_path).expect("metadata bytes"))
            .expect("metadata JSON");
    assert_eq!(metadata["schema_version"], 4);
    assert_eq!(metadata["traffic_path"], "synthetic_city.rne.traffic.json");
    assert_eq!(metadata["buildings"][0]["source_id"], "bldg-A");
    assert_eq!(metadata["buildings"][1]["source_id"], "bldg-B");
    assert_eq!(metadata["buildings"][0]["name"], "RNE City Hall");
    assert_eq!(metadata["roads"][0]["source_id"], "road-main");
    assert_eq!(metadata["roads"][0]["name"], "RNE Avenue");
    assert_eq!(metadata["roads"][0]["lanes"].as_array().unwrap().len(), 2);
    assert_eq!(
        metadata["roads"][0]["lanes"][0]["travel_direction"],
        "principal_axis_positive"
    );
    assert_eq!(metadata["roads"][0]["class"], Value::Null);
    assert_eq!(metadata["roads"][0]["functions"][0], "1");

    let traffic = load_traffic_asset(&result.traffic_path).expect("load generated traffic");
    assert_eq!(traffic.network.lanes.len(), 2);
    assert!(traffic.network.lanes.iter().all(|lane| {
        lane.provenance.authority == AuthorityClass::Derived
            && lane.provenance.accuracy.class == AccuracyClass::Heuristic
            && lane.kind == LaneKind::Driving
            && lane.road_functions == ["1"]
    }));

    let bundle = load_scene_bundle(&result.scene_path).expect("validate generated scene");
    assert_eq!(bundle.scene.objects.len(), 3);
    let mut world = World::new();
    spawn_scene_bundle(&mut world, &bundle, None, SpawnSceneOptions::default())
        .expect("headless spawn generated scene");
    assert_eq!(world.query::<&Visual>().iter(&world).count(), 3);
    assert_eq!(world.query::<&Collider>().iter(&world).count(), 3);

    fs::remove_dir_all(output).expect("remove test output");
}

#[test]
fn imports_lod2_semantics_and_parameterized_texture() {
    let root = temp_output("lod2-textured");
    reset_dir(&root);
    let source = root.join("source");
    let appearance = source.join("appearance");
    fs::create_dir_all(&appearance).expect("create appearance source");
    let citygml_path = source.join("textured.gml");
    fs::copy(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plateau_lod2_textured.gml"),
        &citygml_path,
    )
    .expect("copy CityGML fixture");
    let texture = image::RgbaImage::from_fn(2, 2, |x, y| match (x, y) {
        (0, 0) => image::Rgba([220, 80, 50, 255]),
        (1, 0) => image::Rgba([240, 210, 120, 255]),
        (0, 1) => image::Rgba([60, 100, 180, 255]),
        _ => image::Rgba([230, 230, 220, 255]),
    });
    texture
        .save(appearance.join("facade.png"))
        .expect("write synthetic CC0 texture");

    let output = root.join("output");
    let result = import_citygml_file(
        &citygml_path,
        &output,
        &ImportOptions {
            tile_name: "lod2-textured".into(),
            ..ImportOptions::default()
        },
    )
    .expect("import LOD2 Appearance fixture");

    assert_eq!(result.building_count, 1);
    assert_eq!(result.lod2_building_count, 1);
    assert_eq!(result.textured_surface_count, 1);
    assert_eq!(result.triangle_count, 6);
    let metadata: Value =
        serde_json::from_slice(&fs::read(&result.metadata_path).expect("metadata bytes"))
            .expect("metadata JSON");
    assert_eq!(metadata["schema_version"], 4);
    assert_eq!(metadata["buildings"][0]["lod"], 2);
    assert_eq!(metadata["buildings"][0]["surface_counts"]["wall"], 1);
    assert_eq!(metadata["buildings"][0]["surface_counts"]["roof"], 1);
    assert_eq!(
        metadata["buildings"][0]["texture_paths"][0],
        "textures/appearance_0000.png"
    );

    let mesh_path = output.join("meshes/plateau_building_0000_lod2_building.obj");
    let mesh_parts = rne_render::load_mesh_parts(&mesh_path).expect("load generated OBJ and MTL");
    assert_eq!(mesh_parts.len(), 3);
    assert_eq!(
        mesh_parts
            .iter()
            .filter(|part| part.base_color_texture.is_some())
            .count(),
        1
    );
    assert!(mesh_parts
        .iter()
        .all(|part| part.mesh.texcoords.len() == part.mesh.positions.len()));

    fs::remove_dir_all(root).expect("remove LOD2 output");
}

#[test]
fn rejects_appearance_path_traversal() {
    let root = temp_output("appearance-traversal");
    reset_dir(&root);
    let source = root.join("source");
    fs::create_dir_all(&source).expect("create source");
    let fixture = fs::read_to_string(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plateau_lod2_textured.gml"),
    )
    .expect("read fixture")
    .replace("appearance/facade.png", "../facade.png");
    let citygml_path = source.join("unsafe.gml");
    fs::write(&citygml_path, fixture).expect("write unsafe fixture");
    image::RgbaImage::from_pixel(1, 1, image::Rgba([255, 255, 255, 255]))
        .save(root.join("facade.png"))
        .expect("write outside texture");

    let error = import_citygml_file(
        &citygml_path,
        &root.join("output"),
        &ImportOptions::default(),
    )
    .expect_err("path traversal must fail");
    assert!(matches!(error, ImportError::InvalidTexture { .. }));

    fs::remove_dir_all(root).expect("remove traversal output");
}

#[test]
fn repeated_import_is_byte_for_byte_deterministic() {
    let first = temp_output("determinism-first");
    let second = temp_output("determinism-second");
    reset_dir(&first);
    reset_dir(&second);
    let options = ImportOptions {
        tile_name: "deterministic-tile".into(),
        world_seed: 42,
        ..ImportOptions::default()
    };
    let first_result =
        import_citygml_file(&fixture_path(), &first, &options).expect("first import");
    let second_result =
        import_citygml_file(&fixture_path(), &second, &options).expect("second import");

    assert_eq!(
        fs::read(first_result.scene_path).expect("first scene"),
        fs::read(second_result.scene_path).expect("second scene")
    );
    assert_eq!(
        fs::read(first_result.metadata_path).expect("first metadata"),
        fs::read(second_result.metadata_path).expect("second metadata")
    );
    assert_eq!(
        fs::read(first_result.traffic_path).expect("first traffic"),
        fs::read(second_result.traffic_path).expect("second traffic")
    );
    for mesh in [
        "plateau_building_0000_bldg_a.obj",
        "plateau_building_0001_bldg_b.obj",
        "plateau_road_0000_road_main.obj",
    ] {
        assert_eq!(
            fs::read(first.join("meshes").join(mesh)).expect("first mesh"),
            fs::read(second.join("meshes").join(mesh)).expect("second mesh")
        );
    }

    fs::remove_dir_all(first).expect("remove first output");
    fs::remove_dir_all(second).expect("remove second output");
}

#[test]
fn imports_lod2_and_lod31_road_semantics_into_traffic_schema() {
    let output = temp_output("road-semantics");
    let repeated_output = temp_output("road-semantics-repeat");
    reset_dir(&output);
    reset_dir(&repeated_output);
    let fixture =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plateau_road_semantics.gml");

    let result = import_citygml_file(
        &fixture,
        &output,
        &ImportOptions {
            tile_name: "semantic-roads".into(),
            coordinate_mode: CoordinateMode::ProjectedMeters,
            ..ImportOptions::default()
        },
    )
    .expect("import semantic roads");

    assert_eq!(result.road_count, 2);
    assert_eq!(result.traffic_area_count, 6);
    assert_eq!(result.auxiliary_traffic_area_count, 2);
    assert_eq!(result.lod31_lane_area_count, 2);
    assert_eq!(result.lane_count, 5);
    assert_eq!(result.road_semantics[0].road_source_id, "road-lod2");
    assert_eq!(result.road_semantics[0].class.as_deref(), Some("1"));
    assert_eq!(result.road_semantics[0].functions, ["1", "2"]);
    assert_eq!(
        result
            .traffic_areas
            .iter()
            .find(|area| area.area_source_id == "lod31-lane-a")
            .expect("LOD3.1 lane")
            .functions,
        ["1010"]
    );

    let traffic = load_traffic_asset(&result.traffic_path).expect("load traffic schema");
    assert_eq!(traffic.network.lanes.len(), 5);
    assert_eq!(
        traffic
            .network
            .lanes
            .iter()
            .filter(|lane| lane.kind == LaneKind::Driving)
            .count(),
        4
    );
    assert_eq!(
        traffic
            .network
            .lanes
            .iter()
            .filter(|lane| lane.kind == LaneKind::Sidewalk)
            .count(),
        1
    );
    let explicit_lane = traffic
        .network
        .lanes
        .iter()
        .find(|lane| lane.id.as_str().contains("lod31-lane-a"))
        .expect("explicit LOD3.1 lane");
    assert_eq!(explicit_lane.width_m, 3.0);
    assert_eq!(explicit_lane.road_class.as_deref(), Some("1"));
    assert_eq!(explicit_lane.road_functions, ["3"]);
    assert_eq!(explicit_lane.provenance.authority, AuthorityClass::Derived);
    assert!(explicit_lane
        .provenance
        .method
        .as_deref()
        .expect("derivation method")
        .contains("code 1010"));

    let metadata: Value =
        serde_json::from_slice(&fs::read(&result.metadata_path).expect("metadata bytes"))
            .expect("metadata JSON");
    assert_eq!(metadata["schema_version"], 4);
    assert_eq!(metadata["roads"][0]["lod"], 2);
    assert_eq!(metadata["roads"][1]["lod"], 3);
    assert_eq!(
        metadata["roads"][1]["traffic_areas"][1]["functions"][0],
        "1010"
    );

    let repeated = import_citygml_file(
        &fixture,
        &repeated_output,
        &ImportOptions {
            tile_name: "semantic-roads".into(),
            coordinate_mode: CoordinateMode::ProjectedMeters,
            ..ImportOptions::default()
        },
    )
    .expect("repeat semantic import");
    assert_eq!(
        fs::read(&result.traffic_path).expect("first semantic traffic"),
        fs::read(&repeated.traffic_path).expect("repeated semantic traffic")
    );
    assert_eq!(
        fs::read(&result.metadata_path).expect("first semantic metadata"),
        fs::read(&repeated.metadata_path).expect("repeated semantic metadata")
    );

    fs::remove_dir_all(output).expect("remove semantic output");
    fs::remove_dir_all(repeated_output).expect("remove repeated semantic output");
}
