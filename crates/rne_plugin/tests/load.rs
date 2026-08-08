//! Integration tests for loading controller plugins through the C ABI.

use rne_plugin::{
    load_controller_library, ControllerPlugin, PluginLoadError, VelocityServoController,
};
use std::path::{Path, PathBuf};

fn target_dir() -> PathBuf {
    if let Some(dir) = std::env::var_os("CARGO_TARGET_DIR") {
        return PathBuf::from(dir);
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target")
}

fn library_file_name() -> String {
    if cfg!(target_os = "windows") {
        "rne_plugin_example_velocity_servo.dll".to_string()
    } else if cfg!(target_os = "macos") {
        "librne_plugin_example_velocity_servo.dylib".to_string()
    } else {
        "librne_plugin_example_velocity_servo.so".to_string()
    }
}

/// Locates the example plugin shared library, building it on demand.
///
/// The artifact is produced by `cargo build` on the example crate. A plain
/// workspace build places it under `target/debug/`; `cargo nextest run` does
/// not emit `cdylib` artifacts for workspace members, so the helper falls back
/// to building the crate itself (which also populates `target/debug/`).
fn find_example_library() -> PathBuf {
    let debug = target_dir().join("debug");
    let direct = debug.join(library_file_name());
    let find_in_deps = || -> Option<PathBuf> {
        let deps = debug.join("deps");
        let prefix = if cfg!(target_os = "windows") {
            "rne_plugin_example_velocity_servo-"
        } else {
            "librne_plugin_example_velocity_servo-"
        };
        let extension = library_file_name()
            .rsplit_once('.')
            .map(|(_, extension)| format!(".{extension}"))
            .expect("library extension");
        let entries = std::fs::read_dir(&deps).ok()?;
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().into_owned();
            if name.starts_with(prefix) && name.ends_with(extension.as_str()) {
                return Some(entry.path());
            }
        }
        None
    };
    if direct.exists() {
        return direct;
    }
    if let Some(found) = find_in_deps() {
        return found;
    }
    let status = std::process::Command::new("cargo")
        .arg("build")
        .arg("-p")
        .arg("rne_plugin_example_velocity_servo")
        .current_dir(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .status()
        .expect("run cargo to build the example plugin");
    assert!(
        status.success(),
        "cargo build -p rne_plugin_example_velocity_servo failed"
    );
    if direct.exists() {
        return direct;
    }
    if let Some(found) = find_in_deps() {
        return found;
    }
    panic!(
        "example plugin library still missing after a build; target dir: {}",
        debug.display()
    );
}

fn load() -> Box<dyn ControllerPlugin> {
    load_controller_library(&find_example_library(), "shoulder_joint", 1.0, 2.0, 5.0)
        .expect("load example plugin")
}

#[test]
fn loads_the_example_plugin_and_matches_the_built_in_policy() {
    let loaded = load();
    assert_eq!(loaded.name(), "rne_plugin_example_velocity_servo");

    let built_in = VelocityServoController::new("velocity_servo", "shoulder_joint", 1.0, 2.0, 5.0)
        .expect("built-in controller");

    for (names, positions) in [
        (vec!["shoulder_joint"], vec![0.25]),
        (vec!["shoulder_joint"], vec![-10.0]),
        (vec!["shoulder_joint"], vec![1.0]),
        (vec!["other_joint"], vec![0.5]),
        (vec!["other_joint", "shoulder_joint"], vec![0.5, 0.5]),
    ] {
        assert_eq!(
            loaded.joint_velocity_commands(&names, &positions),
            built_in.joint_velocity_commands(&names, &positions),
            "loaded plugin must match the built-in velocity-servo policy"
        );
    }
}

#[test]
fn invalid_create_parameters_are_rejected() {
    let error = load_controller_library(&find_example_library(), "shoulder_joint", 1.0, -1.0, 5.0)
        .expect_err("negative gain must be rejected");
    assert!(
        matches!(error, PluginLoadError::Create { .. }),
        "expected a create error, got {error}"
    );
}

#[test]
fn missing_library_is_rejected() {
    let error = load_controller_library(
        Path::new("target/does-not-exist/plugin.so"),
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
    .expect_err("missing library must be rejected");
    assert!(
        matches!(error, PluginLoadError::Open { .. }),
        "expected an open error, got {error}"
    );
}
