use rne_assets::{load_scene_bundle, spawn_scene_bundle, SpawnSceneOptions};
use rne_ecs::World;
use rne_physics::Collider;
use rne_plateau::{import_citygml_file, CoordinateMode, ImportOptions};
use rne_render::Visual;
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
    assert_eq!(result.road_count, 1);
    assert_eq!(result.lane_count, 2);
    assert_eq!(result.triangle_count, 26);
    assert_eq!(result.coordinate_mode, CoordinateMode::GeographicDegrees);

    let metadata: Value =
        serde_json::from_slice(&fs::read(&result.metadata_path).expect("metadata bytes"))
            .expect("metadata JSON");
    assert_eq!(metadata["schema_version"], 2);
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
