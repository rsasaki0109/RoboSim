//! Compatibility gate between the latest runtime and the frozen ABI v2 plugin.

use rne_plugin::{load_controller_library, PluginKind, PluginManifest};
use std::path::PathBuf;

const FIXTURE_PACKAGE: &str = "rne_plugin_abi_v2_fixture";
const FIXTURE_NAME: &str = "frozen_velocity_servo_v2";

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn target_dir() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| workspace_root().join("target"))
}

fn library_file_name() -> &'static str {
    if cfg!(target_os = "windows") {
        "rne_plugin_abi_v2_fixture.dll"
    } else if cfg!(target_os = "macos") {
        "librne_plugin_abi_v2_fixture.dylib"
    } else {
        "librne_plugin_abi_v2_fixture.so"
    }
}

fn build_fixture_library() -> PathBuf {
    let debug_dir = target_dir().join("debug");
    let direct = debug_dir.join(library_file_name());
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let status = std::process::Command::new(cargo)
        .args(["build", "--locked", "-p", FIXTURE_PACKAGE])
        .current_dir(workspace_root())
        .status()
        .expect("run cargo to build the frozen ABI v2 fixture");
    assert!(status.success(), "failed to build {FIXTURE_PACKAGE}");
    assert!(
        direct.exists(),
        "frozen ABI v2 fixture library missing after build: {}",
        direct.display()
    );
    direct
}

#[test]
fn latest_runtime_loads_and_steps_the_frozen_abi_v2_plugin() {
    let manifest_path = workspace_root()
        .join("crates")
        .join(FIXTURE_PACKAGE)
        .join("rne-plugin.json");
    let manifest_text = std::fs::read_to_string(&manifest_path).expect("read fixture manifest");
    let manifest: PluginManifest =
        serde_json::from_str(&manifest_text).expect("parse fixture manifest");
    assert_eq!(manifest.name, FIXTURE_NAME);
    assert_eq!(manifest.kind, PluginKind::Controller);
    manifest.validate().expect("fixture manifest validates");

    let plugin = load_controller_library(&build_fixture_library(), "shoulder_joint", 1.0, 2.0, 5.0)
        .expect("the latest runtime must load the frozen ABI v2 fixture");
    assert_eq!(plugin.name(), FIXTURE_NAME);
    assert_eq!(
        plugin.joint_velocity_commands(&["shoulder_joint"], &[0.25]),
        vec![("shoulder_joint".to_string(), 1.5)]
    );
    assert_eq!(
        plugin.joint_velocity_commands(&["shoulder_joint"], &[-10.0]),
        vec![("shoulder_joint".to_string(), 5.0)]
    );
    assert!(plugin
        .joint_velocity_commands(&["other_joint"], &[0.25])
        .is_empty());
}
