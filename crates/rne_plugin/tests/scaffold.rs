//! End-to-end scaffold tests: generate, build, and load a controller plugin.

use rne_plugin::{
    load_controller_library, run_controller_plugin_conformance, scaffold_controller_plugin,
    ControllerPluginConformanceConfig,
};
use std::path::PathBuf;
use std::process::Command;

fn test_dir(suffix: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../target/runs")
        .join(format!("scaffold-{suffix}"))
}

fn shared_library_file_name(name: &str) -> String {
    if cfg!(target_os = "windows") {
        format!("{name}.dll")
    } else if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

#[test]
fn scaffolded_plugin_builds_loads_and_drives() {
    let parent = test_dir("build");
    let _ = std::fs::remove_dir_all(&parent);
    let name = "scaffold_e2e";
    let crate_dir = scaffold_controller_plugin(name, &parent).expect("scaffold");

    let status = Command::new("cargo")
        .arg("build")
        .arg("--offline")
        .arg("--manifest-path")
        .arg(crate_dir.join("Cargo.toml"))
        .arg("--target-dir")
        .arg(crate_dir.join("target"))
        .env_remove("CARGO_TARGET_DIR")
        .env("RUSTFLAGS", "-Dwarnings")
        .status()
        .expect("run cargo build");
    assert!(
        status.success(),
        "the scaffolded crate must compile without edits"
    );

    let library = crate_dir
        .join("target/debug")
        .join(shared_library_file_name(name));
    assert!(
        library.exists(),
        "the scaffolded crate must produce a shared library at {}",
        library.display()
    );

    let plugin =
        load_controller_library(&library, "shoulder_joint", 1.0, 2.0, 5.0).expect("load plugin");
    assert_eq!(plugin.name(), name);
    let commands = plugin.joint_velocity_commands(&["shoulder_joint"], &[0.25]);
    assert_eq!(commands, vec![("shoulder_joint".to_string(), 1.5)]);
    drop(plugin);

    let report = run_controller_plugin_conformance(
        &library,
        &crate_dir.join("rne-plugin.json"),
        &ControllerPluginConformanceConfig {
            joint: "shoulder_joint".to_string(),
            ..ControllerPluginConformanceConfig::default()
        },
    )
    .expect("run conformance against scaffold");
    assert!(report.passed(), "scaffold checks: {:#?}", report.checks);

    let _ = std::fs::remove_dir_all(&parent);
}

#[test]
fn scaffolded_plugin_manifest_is_loadable() {
    let parent = test_dir("manifest");
    let _ = std::fs::remove_dir_all(&parent);
    let name = "scaffold_manifest";
    let crate_dir = scaffold_controller_plugin(name, &parent).expect("scaffold");
    let manifest_path = crate_dir.join("rne-plugin.json");
    let text = std::fs::read_to_string(&manifest_path).expect("read manifest");
    let manifest: rne_plugin::PluginManifest = serde_json::from_str(&text).expect("parse manifest");
    assert_eq!(manifest.name, name);
    assert_eq!(manifest.kind, rne_plugin::PluginKind::Controller);
    manifest.validate().expect("manifest validates");
    let _ = std::fs::remove_dir_all(&parent);
}
