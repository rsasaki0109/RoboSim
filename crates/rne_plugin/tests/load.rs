//! Integration tests for loading controller plugins through the C ABI.

use rne_plugin::{
    load_controller_library, ControllerCapability, ControllerConfiguration, ControllerHost,
    ControllerJointObservation, ControllerLifecycleState, ControllerObservationFrame,
    ControllerPlugin, ControllerResetContext, ControllerRobotObservation, LoadedControllerPlugin,
    PluginLoadError, VelocityServoController, RNE_PLUGIN_ABI_VERSION, RNE_PLUGIN_ABI_VERSION_V2,
};
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

fn target_dir() -> PathBuf {
    if let Some(dir) = std::env::var_os("CARGO_TARGET_DIR") {
        return PathBuf::from(dir);
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target")
}

fn library_file_name(package: &str) -> String {
    if cfg!(target_os = "windows") {
        format!("{package}.dll")
    } else if cfg!(target_os = "macos") {
        format!("lib{package}.dylib")
    } else {
        format!("lib{package}.so")
    }
}

/// Locates the example plugin shared library, building it on demand.
///
/// The artifact is produced by `cargo build` on the example crate. A plain
/// workspace build places it under `target/debug/`; `cargo nextest run` does
/// not emit `cdylib` artifacts for workspace members, so the helper falls back
/// to building the crate itself (which also populates `target/debug/`).
fn find_plugin_library(package: &str) -> PathBuf {
    static CURRENT: OnceLock<PathBuf> = OnceLock::new();
    static LEGACY_V2: OnceLock<PathBuf> = OnceLock::new();
    match package {
        "rne_plugin_example_velocity_servo" => CURRENT
            .get_or_init(|| build_plugin_library(package))
            .clone(),
        "rne_plugin_legacy_v2_fixture" => LEGACY_V2
            .get_or_init(|| build_plugin_library(package))
            .clone(),
        other => build_plugin_library(other),
    }
}

fn build_plugin_library(package: &str) -> PathBuf {
    let debug = target_dir().join("debug");
    let library_file_name = library_file_name(package);
    let direct = debug.join(&library_file_name);
    let find_in_deps = || -> Option<PathBuf> {
        let deps = debug.join("deps");
        let prefix = if cfg!(target_os = "windows") {
            format!("{package}-")
        } else {
            format!("lib{package}-")
        };
        let extension = library_file_name
            .rsplit_once('.')
            .map(|(_, extension)| format!(".{extension}"))
            .expect("library extension");
        let entries = std::fs::read_dir(&deps).ok()?;
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().into_owned();
            if name.starts_with(&prefix) && name.ends_with(extension.as_str()) {
                return Some(entry.path());
            }
        }
        None
    };
    let status = std::process::Command::new("cargo")
        .arg("build")
        .arg("-p")
        .arg(package)
        .current_dir(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .status()
        .expect("run cargo to build the example plugin");
    assert!(status.success(), "cargo build -p {package} failed");
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
    load_controller_library(
        &find_plugin_library("rne_plugin_example_velocity_servo"),
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
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
    let library = find_plugin_library("rne_plugin_example_velocity_servo");
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
    let search_path = find_plugin_library("rne_plugin_example_velocity_servo")
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
    let search_path = find_plugin_library("rne_plugin_example_velocity_servo")
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
    let error = load_controller_library(
        &find_plugin_library("rne_plugin_example_velocity_servo"),
        "shoulder_joint",
        1.0,
        -1.0,
        5.0,
    )
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

fn controller_observation(robot_ids: &[&str]) -> ControllerObservationFrame {
    ControllerObservationFrame::new(
        4,
        40,
        robot_ids
            .iter()
            .map(|robot_id| {
                ControllerRobotObservation::new(
                    *robot_id,
                    vec![ControllerJointObservation::position("shoulder_joint", 0.25)],
                )
                .expect("robot observation")
            })
            .collect(),
    )
    .expect("controller observation")
}

fn reset_context() -> ControllerResetContext {
    ControllerResetContext {
        episode: 0,
        seed: 42,
        step: 0,
        sim_time_ticks: 0,
    }
}

#[test]
fn current_abi_negotiates_lifecycle_and_multi_robot_frames() {
    let loaded = LoadedControllerPlugin::load(
        &find_plugin_library("rne_plugin_example_velocity_servo"),
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
    .expect("load current plugin");
    assert_eq!(loaded.abi_version(), RNE_PLUGIN_ABI_VERSION);
    let mut host = ControllerHost::new(Box::new(loaded)).expect("controller host");
    host.configure(ControllerConfiguration::new([
        ControllerCapability::JointPositionObservation,
        ControllerCapability::JointVelocityCommand,
        ControllerCapability::MultiRobot,
    ]))
    .expect("negotiate current ABI");
    host.activate(reset_context())
        .expect("activate current ABI");
    let action = host
        .step(&controller_observation(&["robot_b", "robot_a"]))
        .expect("step current ABI");
    assert_eq!(action.robots.len(), 2);
    assert!(action.robots.iter().all(|robot| {
        robot.joint_velocities.len() == 1
            && robot.joint_velocities[0].name == "shoulder_joint"
            && robot.joint_velocities[0].velocity_rad_s == 1.5
    }));
    host.shutdown().expect("shutdown current ABI");
    assert_eq!(host.state(), ControllerLifecycleState::Shutdown);
}

#[test]
fn loaded_plugin_serializes_shared_legacy_callbacks_across_threads() {
    let loaded = Arc::new(
        LoadedControllerPlugin::load(
            &find_plugin_library("rne_plugin_example_velocity_servo"),
            "shoulder_joint",
            1.0,
            2.0,
            5.0,
        )
        .expect("load current plugin"),
    );
    let workers = (0..4)
        .map(|_| {
            let loaded = Arc::clone(&loaded);
            std::thread::spawn(move || {
                for _ in 0..16 {
                    assert_eq!(
                        loaded.joint_velocity_commands(&["shoulder_joint"], &[0.25]),
                        [("shoulder_joint".to_string(), 1.5)]
                    );
                }
            })
        })
        .collect::<Vec<_>>();
    for worker in workers {
        worker.join().expect("legacy callback worker");
    }
}

#[test]
fn frozen_abi_v2_plugin_loads_and_steps_in_the_current_runtime() {
    let loaded = LoadedControllerPlugin::load(
        &find_plugin_library("rne_plugin_legacy_v2_fixture"),
        "shoulder_joint",
        1.0,
        2.0,
        5.0,
    )
    .expect("load frozen ABI-v2 fixture");
    assert_eq!(loaded.abi_version(), RNE_PLUGIN_ABI_VERSION_V2);
    assert!(!loaded
        .capabilities()
        .contains(&ControllerCapability::MultiRobot));

    let mut host = ControllerHost::new(Box::new(loaded)).expect("legacy controller host");
    host.configure(ControllerConfiguration::new([
        ControllerCapability::JointPositionObservation,
        ControllerCapability::JointVelocityCommand,
    ]))
    .expect("negotiate legacy ABI");
    host.activate(reset_context()).expect("activate legacy ABI");
    let action = host
        .step(&controller_observation(&["robot"]))
        .expect("step legacy ABI");
    assert_eq!(action.robots[0].joint_velocities[0].velocity_rad_s, 1.5);
    host.shutdown().expect("shutdown legacy ABI");
}

#[test]
fn committed_plugin_manifests_match_their_binary_names() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    for (relative, expected_name) in [
        (
            "crates/rne_plugin_example_velocity_servo/rne-plugin.json",
            "velocity_servo",
        ),
        (
            "crates/rne_plugin_legacy_v2_fixture/rne-plugin.json",
            "legacy_velocity_servo_v2",
        ),
    ] {
        let text = std::fs::read_to_string(root.join(relative)).expect("read plugin manifest");
        let manifest: rne_plugin::PluginManifest =
            serde_json::from_str(&text).expect("parse plugin manifest");
        manifest.validate().expect("valid plugin manifest");
        assert_eq!(manifest.name, expected_name);
    }
}
