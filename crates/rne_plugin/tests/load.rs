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

/// Expected `(joint names, positions, velocity commands)` for the policy tests.
type ExpectedCommands = Vec<(&'static [&'static str], &'static [f64], Vec<(String, f64)>)>;

fn expected_commands() -> ExpectedCommands {
    vec![
        (
            &["shoulder_joint"],
            &[0.25],
            vec![("shoulder_joint".into(), 1.5)],
        ),
        (
            &["shoulder_joint"],
            &[-10.0],
            vec![("shoulder_joint".into(), 5.0)],
        ),
        (
            &["shoulder_joint"],
            &[1.0],
            vec![("shoulder_joint".into(), 0.0)],
        ),
        (&["other_joint"], &[0.5], vec![]),
        (
            &["other_joint", "shoulder_joint"],
            &[0.5, 0.5],
            vec![("shoulder_joint".into(), 1.0)],
        ),
    ]
}

#[test]
fn loads_the_example_plugin_and_matches_the_built_in_policy() {
    let loaded = load();
    assert_eq!(loaded.name(), "velocity_servo");

    let built_in = VelocityServoController::new("velocity_servo", "shoulder_joint", 1.0, 2.0, 5.0)
        .expect("built-in controller");

    for (names, positions, expected) in expected_commands() {
        assert_eq!(
            loaded.joint_velocity_commands(names, positions),
            expected,
            "loaded plugin must match the built-in velocity-servo policy"
        );
        assert_eq!(built_in.joint_velocity_commands(names, positions), expected);
    }
}

#[test]
fn discovers_the_plugin_by_name_in_a_search_directory() {
    let library = find_example_library();
    let search_path = library
        .parent()
        .expect("library parent directory")
        .to_path_buf();
    let discovered = rne_plugin::discover_controller_plugin(
        "velocity_servo",
        &[search_path.as_path()],
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
    .expect("discover velocity_servo");

    assert_eq!(discovered.name(), "velocity_servo");
    for (names, positions, expected) in expected_commands() {
        assert_eq!(
            discovered.joint_velocity_commands(names, positions),
            expected,
            "discovered plugin must match the velocity-servo policy"
        );
    }
}

#[test]
fn discovery_falls_back_to_the_built_in_without_search_paths() {
    let discovered = rne_plugin::discover_controller_plugin(
        "velocity_servo",
        &[],
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
    .expect("built-in fallback");
    assert_eq!(discovered.name(), "velocity_servo");
    for (names, positions, expected) in expected_commands() {
        assert_eq!(
            discovered.joint_velocity_commands(names, positions),
            expected,
            "built-in fallback must match the velocity-servo policy"
        );
    }
}

#[test]
fn discovery_rejects_an_unknown_plugin_name() {
    let search_path = find_example_library()
        .parent()
        .expect("library parent directory")
        .to_path_buf();
    let error = rne_plugin::discover_controller_plugin(
        "nonexistent_controller",
        &[search_path.as_path()],
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
    .expect_err("unknown plugin must be rejected");
    assert!(
        matches!(error, PluginLoadError::NotFound { .. }),
        "expected a not-found error, got {error}"
    );
}

#[test]
fn enumerates_available_plugin_names() {
    let search_path = find_example_library()
        .parent()
        .expect("library parent directory")
        .to_path_buf();
    let found =
        rne_plugin::discover_plugin_names(&[search_path.as_path()]).expect("discover names");
    assert!(
        found.iter().any(|(name, _)| name == "velocity_servo"),
        "the example plugin must be discovered by name, got {found:?}"
    );
    let mut names = found
        .iter()
        .map(|(name, _)| name.as_str())
        .collect::<Vec<_>>();
    names.sort_unstable();
    assert!(names.windows(2).all(|pair| pair[0] <= pair[1]));
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
