//! Versioned headless simulation run manifests.

use crate::error::AssetError;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Current `.rne.run.toml` schema version.
pub const RUN_MANIFEST_VERSION: u32 = 1;

/// Versioned configuration for one headless simulation run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunManifest {
    /// Schema version, currently [`RUN_MANIFEST_VERSION`].
    pub version: u32,
    /// Scene asset path, relative to this manifest unless absolute.
    ///
    /// Required unless a [`RunScenario`] is configured.
    #[serde(default)]
    pub scene: PathBuf,
    /// Optional replacement for the scene's world seed.
    #[serde(default)]
    pub seed: Option<u64>,
    /// Fixed-step simulation clock configuration.
    #[serde(default)]
    pub clock: RunClock,
    /// Controller configuration for the first runner boundary.
    #[serde(default)]
    pub controller: RunController,
    /// Sensor subscriptions requesting full typed payload capture.
    #[serde(default)]
    pub sensors: Vec<RunSensorSubscription>,
    /// Optional OpenSCENARIO scenario that replaces the fixed-step physics run.
    #[serde(default)]
    pub scenario: Option<RunScenario>,
    /// Physics backend requirements verified before the run starts.
    #[serde(default)]
    pub physics: RunPhysics,
    /// Output and replay checks for this run.
    #[serde(default)]
    pub output: RunOutput,
}

impl RunManifest {
    /// Resolves [`Self::scene`] against the manifest's parent directory.
    pub fn resolve_scene_path(&self, manifest_path: &Path) -> PathBuf {
        if self.scene.is_absolute() {
            self.scene.clone()
        } else {
            manifest_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(&self.scene)
        }
    }

    /// Resolves a relative output path against the manifest's parent directory.
    pub fn resolve_output_path(&self, manifest_path: &Path, output_path: &Path) -> PathBuf {
        if output_path.is_absolute() {
            output_path.to_path_buf()
        } else {
            manifest_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(output_path)
        }
    }
}

/// Fixed-step clock settings in a [`RunManifest`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunClock {
    /// Number of fixed simulation steps.
    #[serde(default = "default_run_steps")]
    pub steps: u64,
    /// Fixed simulation rate in hertz.
    #[serde(default = "default_run_hz")]
    pub hz: f64,
}

impl Default for RunClock {
    fn default() -> Self {
        Self {
            steps: default_run_steps(),
            hz: default_run_hz(),
        }
    }
}

/// Controller kinds supported by the version 1 runner boundary.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunControllerKind {
    /// Do not inject actuator commands.
    #[default]
    None,
    /// Apply one wheel velocity to every differential-drive robot.
    DifferentialDrive,
    /// Apply one named joint velocity to every matching joint.
    JointVelocity,
    /// Apply one named joint effort to every matching joint.
    JointEffort,
    /// Interpolate per-joint position trajectories over simulation time.
    JointTrajectory,
}

/// Controller settings in a [`RunManifest`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunController {
    /// Controller implementation selected by the runner.
    #[serde(default)]
    pub kind: RunControllerKind,
    /// Wheel velocity command used by [`RunControllerKind::DifferentialDrive`].
    #[serde(default)]
    pub wheel_velocity_rad_s: f64,
    /// URDF / ECS joint name used by the named joint controller kinds.
    #[serde(default)]
    pub joint: String,
    /// Joint velocity command in radians per second.
    #[serde(default)]
    pub velocity_rad_s: f64,
    /// Joint effort command in newton-meters.
    #[serde(default)]
    pub effort_nm: f64,
    /// Per-joint position waypoints used by [`RunControllerKind::JointTrajectory`].
    #[serde(default)]
    pub joint_trajectories: Vec<RunJointTrajectory>,
}

impl Default for RunController {
    fn default() -> Self {
        Self {
            kind: RunControllerKind::None,
            wheel_velocity_rad_s: 0.0,
            joint: String::new(),
            velocity_rad_s: 0.0,
            effort_nm: 0.0,
            joint_trajectories: Vec::new(),
        }
    }
}

/// One time-indexed position waypoint in a [`RunJointTrajectory`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunTrajectoryWaypoint {
    /// Simulation time since the run start, in seconds.
    pub t_s: f64,
    /// Joint position target in radians.
    pub position_rad: f64,
}

/// Position waypoints for one named joint over simulation time.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunJointTrajectory {
    /// URDF / ECS joint name.
    pub joint: String,
    /// Sorted time-indexed position waypoints.
    pub waypoints: Vec<RunTrajectoryWaypoint>,
}

/// Sensor kinds that a [`RunSensorSubscription`] can select.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunSensorKind {
    /// Inertial measurement unit.
    #[default]
    Imu,
    /// Scanning LiDAR.
    Lidar,
    /// RGB(-D) camera.
    Camera,
    /// Wheel encoder.
    WheelEncoder,
}

/// Selects one or more sensors for full typed payload capture.
///
/// A subscription matches every sensor whose entity name equals [`Self::name`]
/// or whose kind equals [`Self::kind`]. At least one selector is required.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunSensorSubscription {
    /// Sensor entity name, for example `lidar` or `wrist_camera`.
    #[serde(default)]
    pub name: Option<String>,
    /// Sensor kind selector.
    #[serde(default)]
    pub kind: Option<RunSensorKind>,
}

/// Output settings in a [`RunManifest`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunOutput {
    /// Repeat the run and require the final report to match exactly.
    #[serde(default)]
    pub determinism_check: bool,
    /// Optional `.rne-replay` output path, relative to the manifest.
    #[serde(default)]
    pub replay_path: Option<PathBuf>,
}

/// OpenSCENARIO scenario settings in a [`RunManifest`].
///
/// When present, the runner executes the scenario over the traffic runtime
/// instead of the fixed-step physics simulation. The manifest `scene` is then
/// optional; the road network comes from the scenario's `LogicFile`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunScenario {
    /// OpenSCENARIO `.xosc` path, relative to this manifest unless absolute.
    pub xosc: PathBuf,
}

impl RunScenario {
    /// Resolves [`Self::xosc`] against the manifest's parent directory.
    pub fn resolve_xosc_path(&self, manifest_path: &Path) -> PathBuf {
        if self.xosc.is_absolute() {
            self.xosc.clone()
        } else {
            manifest_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(&self.xosc)
        }
    }
}

/// Physics capability names a run manifest can require.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunPhysicsCapability {
    /// Rigid body simulation.
    RigidBody,
    /// Articulated (multibody) bodies.
    Articulation,
    /// GPU rigid body simulation.
    GpuRigidBody,
    /// Deterministic stepping.
    DeterministicStep,
    /// Soft bodies.
    SoftBody,
    /// Contact force reporting.
    ContactForce,
    /// Batched raycasts.
    RaycastBatch,
}

/// Physics backend requirements verified before a run starts.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunPhysics {
    /// Capabilities the physics backend must provide, deduplicated on parse.
    #[serde(default)]
    pub required_capabilities: Vec<RunPhysicsCapability>,
}

/// Loads and validates a `.rne.run.toml` manifest from disk.
pub fn load_run_manifest(path: &Path) -> Result<RunManifest, AssetError> {
    let text = fs::read_to_string(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_run_manifest(path, &text)
}

/// Parses and validates a run manifest from TOML text.
pub fn parse_run_manifest(path: &Path, text: &str) -> Result<RunManifest, AssetError> {
    let manifest: RunManifest = toml::from_str(text).map_err(|error| {
        AssetError::invalid(path.display().to_string(), format!("TOML: {error}"))
    })?;
    validate_run_manifest(path, manifest)
}

fn validate_run_manifest(path: &Path, manifest: RunManifest) -> Result<RunManifest, AssetError> {
    if manifest.version != RUN_MANIFEST_VERSION {
        return Err(AssetError::invalid(
            path.display().to_string(),
            format!(
                "unsupported run manifest version {}; expected {}",
                manifest.version, RUN_MANIFEST_VERSION
            ),
        ));
    }
    if manifest.scenario.is_none() && manifest.scene.as_os_str().is_empty() {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "scene path must not be empty unless a scenario is configured",
        ));
    }
    if let Some(scenario) = &manifest.scenario {
        if scenario.xosc.as_os_str().is_empty() {
            return Err(AssetError::invalid(
                path.display().to_string(),
                "scenario.xosc must not be empty",
            ));
        }
    }
    if !manifest.clock.hz.is_finite() || manifest.clock.hz <= 0.0 {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "clock.hz must be finite and positive",
        ));
    }
    if !manifest.controller.wheel_velocity_rad_s.is_finite() {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "controller.wheel_velocity_rad_s must be finite",
        ));
    }
    if !manifest.controller.velocity_rad_s.is_finite() {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "controller.velocity_rad_s must be finite",
        ));
    }
    if !manifest.controller.effort_nm.is_finite() {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "controller.effort_nm must be finite",
        ));
    }
    if matches!(
        manifest.controller.kind,
        RunControllerKind::JointVelocity | RunControllerKind::JointEffort
    ) && manifest.controller.joint.trim().is_empty()
    {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "controller.joint must not be empty for a named joint controller",
        ));
    }
    if manifest.controller.kind == RunControllerKind::JointTrajectory {
        if manifest.controller.joint_trajectories.is_empty() {
            return Err(AssetError::invalid(
                path.display().to_string(),
                "controller.joint_trajectories must not be empty for a joint trajectory controller",
            ));
        }
        let mut joint_names = manifest
            .controller
            .joint_trajectories
            .iter()
            .map(|trajectory| trajectory.joint.clone())
            .collect::<Vec<_>>();
        joint_names.sort_unstable();
        if joint_names.windows(2).any(|window| window[0] == window[1]) {
            return Err(AssetError::invalid(
                path.display().to_string(),
                "controller.joint_trajectories joint names must be unique",
            ));
        }
        for (index, trajectory) in manifest.controller.joint_trajectories.iter().enumerate() {
            if trajectory.joint.trim().is_empty() {
                return Err(AssetError::invalid(
                    path.display().to_string(),
                    format!("controller.joint_trajectories[{index}].joint must not be empty"),
                ));
            }
            if trajectory.waypoints.len() < 2 {
                return Err(AssetError::invalid(
                    path.display().to_string(),
                    format!(
                        "controller.joint_trajectories[{index}] requires at least two waypoints"
                    ),
                ));
            }
            for (waypoint_index, waypoint) in trajectory.waypoints.iter().enumerate() {
                if !waypoint.t_s.is_finite() || waypoint.t_s < 0.0 {
                    return Err(AssetError::invalid(
                        path.display().to_string(),
                        format!(
                            "controller.joint_trajectories[{index}].waypoints[{waypoint_index}].t_s must be finite and non-negative"
                        ),
                    ));
                }
                if !waypoint.position_rad.is_finite() {
                    return Err(AssetError::invalid(
                        path.display().to_string(),
                        format!(
                            "controller.joint_trajectories[{index}].waypoints[{waypoint_index}].position_rad must be finite"
                        ),
                    ));
                }
            }
            if trajectory
                .waypoints
                .windows(2)
                .any(|window| window[0].t_s >= window[1].t_s)
            {
                return Err(AssetError::invalid(
                    path.display().to_string(),
                    format!(
                        "controller.joint_trajectories[{index}] waypoints must be sorted by increasing t_s"
                    ),
                ));
            }
        }
    }
    for (index, subscription) in manifest.sensors.iter().enumerate() {
        if subscription.name.is_none() && subscription.kind.is_none() {
            return Err(AssetError::invalid(
                path.display().to_string(),
                format!("sensors[{index}] must select a sensor by name or kind"),
            ));
        }
        if let Some(name) = subscription.name.as_deref() {
            if name.trim().is_empty() {
                return Err(AssetError::invalid(
                    path.display().to_string(),
                    format!("sensors[{index}].name must not be empty"),
                ));
            }
        }
    }
    let mut required = manifest.physics.required_capabilities.clone();
    required.sort_unstable();
    if required.windows(2).any(|window| window[0] == window[1]) {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "physics.required_capabilities must be unique".to_string(),
        ));
    }
    Ok(manifest)
}

fn default_run_steps() -> u64 {
    600
}

fn default_run_hz() -> f64 {
    60.0
}

#[cfg(test)]
mod tests {
    use super::{
        parse_run_manifest, RunControllerKind, RunPhysicsCapability, RunSensorKind,
        RUN_MANIFEST_VERSION,
    };
    use std::path::{Path, PathBuf};

    #[test]
    fn parses_and_resolves_v1_manifest() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/example.rne.run.toml"),
            r#"
version = 1
scene = "../scenes/mesh_diff_drive.rne.scene.toml"
seed = 7

[clock]
steps = 120
hz = 30.0

[controller]
kind = "differential_drive"
wheel_velocity_rad_s = 4.0

[output]
determinism_check = true
replay_path = "../../target/example.rne-replay"
"#,
        )
        .expect("manifest");
        assert_eq!(manifest.version, RUN_MANIFEST_VERSION);
        assert_eq!(manifest.seed, Some(7));
        assert_eq!(
            manifest.controller.kind,
            RunControllerKind::DifferentialDrive
        );
        assert_eq!(manifest.clock.steps, 120);
        assert_eq!(
            manifest.output.replay_path,
            Some(PathBuf::from("../../target/example.rne-replay"))
        );
        assert_eq!(
            manifest.resolve_output_path(
                Path::new("assets/runs/example.rne.run.toml"),
                Path::new("../../target/example.rne-replay")
            ),
            Path::new("assets/runs/../../target/example.rne-replay")
        );
        assert_eq!(
            manifest.resolve_scene_path(Path::new("assets/runs/example.rne.run.toml")),
            Path::new("assets/runs/../scenes/mesh_diff_drive.rne.scene.toml")
        );
    }

    #[test]
    fn rejects_unknown_version_and_non_finite_clock() {
        let version_error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            "version = 2\nscene = \"scene.rne.scene.toml\"",
        )
        .expect_err("version must be rejected");
        assert!(version_error
            .to_string()
            .contains("unsupported run manifest version"));

        let clock_error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            "version = 1\nscene = \"scene.rne.scene.toml\"\n[clock]\nhz = 0.0",
        )
        .expect_err("clock must be rejected");
        assert!(clock_error.to_string().contains("clock.hz"));
    }

    #[test]
    fn parses_named_joint_velocity_controller() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/joint.rne.run.toml"),
            r#"
version = 1
scene = "../scenes/mm_minimal.rne.scene.toml"

[controller]
kind = "joint_velocity"
joint = "shoulder_joint"
velocity_rad_s = 0.4
"#,
        )
        .expect("joint manifest");

        assert_eq!(manifest.controller.kind, RunControllerKind::JointVelocity);
        assert_eq!(manifest.controller.joint, "shoulder_joint");
        assert_eq!(manifest.controller.velocity_rad_s, 0.4);
    }

    #[test]
    fn parses_sensor_subscriptions_by_name_and_kind() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/sensors.rne.run.toml"),
            r#"
version = 1
scene = "../scenes/mesh_diff_drive.rne.scene.toml"

[[sensors]]
name = "wrist_camera"

[[sensors]]
kind = "lidar"
"#,
        )
        .expect("sensor manifest");

        assert_eq!(manifest.sensors.len(), 2);
        assert_eq!(manifest.sensors[0].name.as_deref(), Some("wrist_camera"));
        assert_eq!(manifest.sensors[0].kind, None);
        assert_eq!(manifest.sensors[1].name, None);
        assert_eq!(manifest.sensors[1].kind, Some(RunSensorKind::Lidar));
    }

    #[test]
    fn rejects_sensor_subscription_without_selector() {
        let error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            "version = 1\nscene = \"scene.rne.scene.toml\"\n\n[[sensors]]\n",
        )
        .expect_err("selector must be required");
        assert!(error
            .to_string()
            .contains("select a sensor by name or kind"));
    }

    #[test]
    fn parses_scenario_run_without_scene() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/scenario.rne.run.toml"),
            r#"
version = 1

[clock]
steps = 300
hz = 60.0

[scenario]
xosc = "../scenarios/speed.xosc"
"#,
        )
        .expect("scenario manifest");

        let scenario = manifest.scenario.expect("scenario configured");
        assert_eq!(scenario.xosc, PathBuf::from("../scenarios/speed.xosc"));
        assert_eq!(
            scenario.resolve_xosc_path(Path::new("assets/runs/scenario.rne.run.toml")),
            Path::new("assets/runs/../scenarios/speed.xosc")
        );
    }

    #[test]
    fn rejects_scenario_without_xosc() {
        let error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            "version = 1\n\n[scenario]\nxosc = \"\"\n",
        )
        .expect_err("xosc must be required");
        assert!(error.to_string().contains("scenario.xosc"));
    }

    #[test]
    fn parses_joint_trajectory_controller() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/trajectory.rne.run.toml"),
            r#"
version = 1
scene = "../scenes/mm_minimal.rne.scene.toml"

[controller]
kind = "joint_trajectory"

[[controller.joint_trajectories]]
joint = "shoulder_joint"
waypoints = [
    { t_s = 0.0, position_rad = 0.0 },
    { t_s = 1.0, position_rad = 0.5 },
    { t_s = 2.0, position_rad = 0.0 },
]
"#,
        )
        .expect("trajectory manifest");

        assert_eq!(manifest.controller.kind, RunControllerKind::JointTrajectory);
        let trajectory = &manifest.controller.joint_trajectories[0];
        assert_eq!(trajectory.joint, "shoulder_joint");
        assert_eq!(trajectory.waypoints.len(), 3);
        assert_eq!(trajectory.waypoints[1].t_s, 1.0);
        assert_eq!(trajectory.waypoints[1].position_rad, 0.5);
    }

    #[test]
    fn rejects_unsorted_trajectory_waypoints() {
        let error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            r#"
version = 1
scene = "scene.rne.scene.toml"

[controller]
kind = "joint_trajectory"

[[controller.joint_trajectories]]
joint = "shoulder_joint"
waypoints = [
    { t_s = 1.0, position_rad = 0.0 },
    { t_s = 0.0, position_rad = 0.5 },
]
"#,
        )
        .expect_err("waypoints must be sorted");
        assert!(error.to_string().contains("sorted by increasing t_s"));
    }

    #[test]
    fn parses_physics_requirements() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/physics.rne.run.toml"),
            r#"
version = 1
scene = "../scenes/mesh_diff_drive.rne.scene.toml"

[physics]
required_capabilities = ["articulation", "contact_force"]
"#,
        )
        .expect("physics manifest");

        assert_eq!(
            manifest.physics.required_capabilities,
            vec![
                RunPhysicsCapability::Articulation,
                RunPhysicsCapability::ContactForce,
            ]
        );
    }

    #[test]
    fn rejects_duplicate_physics_requirements() {
        let error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            r#"
version = 1
scene = "scene.rne.scene.toml"

[physics]
required_capabilities = ["rigid_body", "rigid_body"]
"#,
        )
        .expect_err("duplicates must be rejected");
        assert!(error.to_string().contains("must be unique"));
    }
}
