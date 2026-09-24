//! Local application commands and GPU sensor presentation.

use crate::{
    project::{JointPosition, Project, RobotFormat, RobotSource},
    simulation::Simulation,
};
use anyhow::{ensure, Context, Result};
use rne_math::Vec3;
use rne_render::{Camera, MeshRenderCache, RenderBackend};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Deserialize;
use serde_json::{json, Value};
use std::{collections::BTreeMap, io::Cursor, path::PathBuf};

#[derive(Debug, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
pub(crate) enum Command {
    Step {
        steps: u32,
    },
    Targets {
        targets: BTreeMap<String, JointPosition>,
    },
    SavePose {
        name: String,
    },
    ApplyPose {
        name: String,
    },
    ReplaceProject {
        project: Project,
    },
    Import {
        format: RobotFormat,
        xml: String,
    },
    Preset {
        name: String,
    },
    Reset,
    View {
        yaw_rad: f64,
        pitch_rad: f64,
        distance_m: f64,
    },
}

pub(crate) struct Host {
    simulation: Simulation,
    renderer: WgpuRenderBackend,
    meshes: MeshRenderCache,
    asset_root: PathBuf,
    orbit: CameraOrbit,
}

impl std::fmt::Debug for Host {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Host")
            .field("simulation", &self.simulation)
            .field("asset_root", &self.asset_root)
            .finish_non_exhaustive()
    }
}

impl Host {
    pub(crate) fn new(project: Project, asset_root: PathBuf) -> Result<Self> {
        let simulation = Simulation::new(project, Some(&asset_root))?;
        let (focus, distance_m) = simulation.camera_fit();
        let mut meshes = MeshRenderCache::new();
        meshes.resolve_scene(&mut simulation.render_scene(), &[&asset_root])?;
        Ok(Self {
            simulation,
            renderer: WgpuRenderBackend::new().context("initialize GPU renderer")?,
            meshes,
            asset_root,
            orbit: CameraOrbit {
                yaw_rad: 0.7,
                pitch_rad: 1.0,
                distance_m,
                focus,
            },
        })
    }

    pub(crate) fn command(&mut self, command: Command) -> Result<()> {
        match command {
            Command::Step { steps } => self.simulation.step(steps)?,
            Command::Targets { targets } => self.simulation.set_targets(targets)?,
            Command::SavePose { name } => self.simulation.project.save_pose(&name)?,
            Command::ApplyPose { name } => {
                let pose = self
                    .simulation
                    .project
                    .poses
                    .get(&name)
                    .context("unknown saved pose")?
                    .clone();
                self.simulation.project.apply_pose(&pose)?;
            }
            Command::ReplaceProject { project } => self.replace(project)?,
            Command::Import { format, xml } => {
                self.replace(Project::new(RobotSource { format, xml })?)?
            }
            Command::Preset { name } => self.replace(preset(&name)?)?,
            Command::Reset => self.replace(self.simulation.project.clone())?,
            Command::View {
                yaw_rad,
                pitch_rad,
                distance_m,
            } => {
                ensure!(
                    yaw_rad.is_finite()
                        && yaw_rad.abs() <= 100.0
                        && pitch_rad.is_finite()
                        && (0.15..=1.45).contains(&pitch_rad)
                        && distance_m.is_finite()
                        && (0.3..=30.0).contains(&distance_m),
                    "invalid camera orbit"
                );
                self.orbit.yaw_rad = yaw_rad;
                self.orbit.pitch_rad = pitch_rad;
                self.orbit.distance_m = distance_m;
            }
        }
        Ok(())
    }

    fn replace(&mut self, project: Project) -> Result<()> {
        let refit = project.robot != self.simulation.project.robot
            || project.robot_rotation_rpy_rad != self.simulation.project.robot_rotation_rpy_rad;
        let candidate = Simulation::new(project, Some(&self.asset_root))?;
        let mut meshes = MeshRenderCache::new();
        meshes.resolve_scene(&mut candidate.render_scene(), &[&self.asset_root])?;
        if refit {
            let (focus, distance_m) = candidate.camera_fit();
            self.orbit.focus = focus;
            self.orbit.distance_m = distance_m;
        }
        self.simulation = candidate;
        self.meshes = meshes;
        self.renderer.reset_taa_history();
        Ok(())
    }

    pub(crate) fn snapshot(&mut self) -> Result<Value> {
        let mut scene = self.simulation.render_scene();
        self.meshes.resolve_scene(&mut scene, &[&self.asset_root])?;
        let mut camera = Camera::new(640, 400, 0.9);
        camera.near_m = 0.02;
        camera.far_m = 30.0;
        let view = self.renderer.render_scene_camera(
            &camera,
            &self.orbit.camera_transform(),
            &scene,
            [0.025, 0.04, 0.06, 1.0],
        )?;
        let image = image::RgbaImage::from_raw(640, 400, view.color.rgba8)
            .context("invalid rendered image")?;
        let mut png = Cursor::new(Vec::new());
        image::DynamicImage::ImageRgba8(image).write_to(&mut png, image::ImageFormat::Png)?;
        let mut sensor_camera = Camera::new(160, 120, 1.0);
        sensor_camera.near_m = 0.02;
        sensor_camera.far_m = 10.0;
        let sensor_mount = CameraOrbit {
            yaw_rad: -0.5,
            pitch_rad: 1.1,
            distance_m: 2.0,
            focus: Vec3::new(0.0, 0.35, 0.0),
        }
        .camera_transform();
        let sensor = self.renderer.render_scene_camera(
            &sensor_camera,
            &sensor_mount,
            &scene,
            [0.025, 0.04, 0.06, 1.0],
        )?;
        let depth = sensor
            .depth
            .depth_m
            .iter()
            .map(|v| {
                if v.is_finite() && *v >= 0.0 && *v < sensor_camera.far_m as f32 {
                    Some(*v)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        Ok(json!({
            "project": self.simulation.project,
            "sim_time_ticks": self.simulation.sim_time_ticks().to_string(),
            "state_hash": format!("{:016x}", self.simulation.state_hash()),
            "joints": self.simulation.readings()?,
            "view_png_base64": base64::encode(png.into_inner()),
            "camera": { "sim_time_ticks": self.simulation.sim_time_ticks().to_string(),
                "width": 160, "height": 120, "rgba8_base64": base64::encode(sensor.color.rgba8),
                "depth_m": depth, "near_m": sensor_camera.near_m, "far_m": sensor_camera.far_m },
            "lidar": self.simulation.lidar()?,
            "orbit": { "yaw_rad": self.orbit.yaw_rad, "pitch_rad": self.orbit.pitch_rad,
                "distance_m": self.orbit.distance_m }
        }))
    }
}

pub(crate) fn preset(name: &str) -> Result<Project> {
    let source = match name {
        "so101" => RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../assets/robots/so101/so101.urdf").into(),
        },
        "slider" => RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
                .into(),
        },
        "arm" => RobotSource {
            format: RobotFormat::Mjcf,
            xml: include_str!("../../crates/rne_mjcf/tests/fixtures/two_link_arm.xml").into(),
        },
        _ => anyhow::bail!("unknown robot preset"),
    };
    let mut project = Project::new(source)?;
    if name == "so101" {
        project.robot_rotation_rpy_rad[0] = -std::f64::consts::FRAC_PI_2;
    }
    project.objects.push(crate::project::SceneObject {
        name: "inspection-wall".into(),
        position_m: [1.3, 0.5, -0.4],
        size_m: [0.15, 1.0, 1.2],
        color_rgba: [0.2, 0.55, 0.65, 1.0],
    });
    project.validate()?;
    Ok(project)
}
