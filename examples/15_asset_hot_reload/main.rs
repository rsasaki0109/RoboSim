//! Demonstrates asset validation and hot reload polling.

use rne_assets::{inspect_asset, load_scene_bundle, validate_asset, AssetHotReloader};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

fn main() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = repo_root.join("assets/scenes/episode_diff_drive.rne.scene.toml");

    if env::args().any(|arg| arg == "--smoke") {
        run_smoke(&scene_path);
        return;
    }

    let bundle = load_scene_bundle(&scene_path).expect("load scene bundle");
    println!(
        "loaded scene: robots={} seed={}",
        bundle.robots.len(),
        bundle.scene.world.seed
    );
    println!("{}", inspect_asset(&scene_path).expect("inspect"));

    let mut reloader = AssetHotReloader::load(&scene_path).expect("hot reloader");
    println!("watching dependencies for 3 seconds (edit the scene file to reload)...");

    for _ in 0..6 {
        if reloader.poll().expect("poll") {
            println!(
                "reloaded scene: seed={} robots={}",
                reloader.bundle().scene.world.seed,
                reloader.bundle().robots.len()
            );
        }
        thread::sleep(Duration::from_millis(500));
    }
}

fn run_smoke(scene_path: &Path) {
    validate_asset(scene_path).expect("validate scene");
    let bundle = load_scene_bundle(scene_path).expect("load bundle");
    assert!(!bundle.robots.is_empty());

    let temp_dir =
        env::temp_dir().join(format!("rne_asset_hot_reload_smoke_{}", std::process::id()));
    let _ = fs::remove_dir_all(&temp_dir);
    fs::create_dir_all(&temp_dir).expect("temp dir");

    let robot_src = scene_path
        .parent()
        .expect("scene dir")
        .parent()
        .expect("assets dir")
        .join("robots/diff_drive.rne.robot.toml");
    let robot_dst = temp_dir.join("diff_drive.rne.robot.toml");
    fs::copy(&robot_src, &robot_dst).expect("copy robot asset");

    let temp_scene = temp_dir.join("episode_diff_drive.rne.scene.toml");
    fs::write(
        &temp_scene,
        r#"
[world]
seed = 7

[ground]
enabled = true

[[robots]]
path = "diff_drive.rne.robot.toml"
"#,
    )
    .expect("write temp scene");

    let mut reloader = AssetHotReloader::load(&temp_scene).expect("hot reloader");
    assert!(!reloader.poll().expect("poll"));
    assert_eq!(reloader.bundle().scene.world.seed, 7);

    thread::sleep(Duration::from_millis(1100));
    fs::write(
        &temp_scene,
        r#"
[world]
seed = 8

[ground]
enabled = true

[[robots]]
path = "diff_drive.rne.robot.toml"
"#,
    )
    .expect("rewrite temp scene");

    assert!(reloader.poll().expect("poll"));
    assert_eq!(reloader.bundle().scene.world.seed, 8);

    let _ = fs::remove_dir_all(temp_dir);

    println!(
        "asset hot reload smoke: scene={} robots={}",
        scene_path.display(),
        bundle.robots.len()
    );
}
