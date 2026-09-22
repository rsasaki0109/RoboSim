//! Native fixed-step workbench simulation; no window or GPU is required.

use crate::project::{JointInfo, JointPosition, Project};
use anyhow::{ensure, Context, Result};
use rne_core::{SimClock, SimDuration};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::Vec3;
use rne_physics::{
    hash_physics_state, Collider, JointMotor, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId,
    RaycastQuery, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::{LinkVisuals, RenderScene, Visual, VisualShape};
use rne_urdf_import::{
    attach_urdf_document_articulation, spawn_urdf_document_with_config, UrdfArticulationConfig,
    UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};
use serde::Serialize;
use std::{collections::BTreeMap, path::Path};

const STEP_TICKS: u64 = 4_166_667;

/// One actual measured joint coordinate alongside its requested target.
#[derive(Debug, Serialize)]
pub(crate) struct JointReading {
    pub info: JointInfo,
    pub measured: f64,
    pub velocity: f64,
    pub target: JointPosition,
}

/// A timestamped, instantaneous noiseless scan from an explicit world mount.
#[derive(Debug, Serialize)]
pub(crate) struct LidarReading {
    pub sim_time_ticks: u64,
    pub origin_m: [f64; 3],
    pub max_range_m: f64,
    pub ranges_m: Vec<Option<f64>>,
    pub points_m: Vec<[f64; 3]>,
}

/// Owns all backend handles at the application boundary.
pub(crate) struct Simulation {
    pub project: Project,
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    clock: SimClock,
    joints: Vec<(JointInfo, Entity)>,
    robot_links: Vec<Entity>,
}

impl std::fmt::Debug for Simulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Simulation")
            .field("clock", &self.clock)
            .field("joints", &self.joints)
            .finish_non_exhaustive()
    }
}

impl Simulation {
    pub fn new(project: Project, asset_root: Option<&Path>) -> Result<Self> {
        project.validate()?;
        let document = project.robot.document()?;
        ensure!(
            document.robot.links.len() <= 512 && document.robot.joints.len() <= 512,
            "workbench supports at most 512 links and joints"
        );
        for element in document
            .robot
            .links
            .iter()
            .flat_map(|link| link.visuals.iter().chain(&link.collisions))
        {
            if let rne_urdf_import::UrdfGeometry::Mesh { path, .. } = &element.geometry {
                let root = asset_root
                    .context("mesh model requires --asset-root")?
                    .canonicalize()?;
                let resolved = rne_render::resolve_package_uri(path, &root)
                    .canonicalize()
                    .with_context(|| format!("missing mesh {path}; check --asset-root"))?;
                ensure!(
                    resolved.starts_with(&root),
                    "mesh lies outside --asset-root: {path}"
                );
                ensure!(
                    resolved.metadata()?.len() <= 32 * 1024 * 1024,
                    "mesh exceeds 32 MiB: {path}"
                );
            }
        }
        ensure!(
            document.robot.joints.iter().all(|j| j.mimic.is_none()),
            "physics preview does not yet support mimic joints"
        );
        let mut world = World::new();
        let spawned = spawn_urdf_document_with_config(
            &mut world,
            &document,
            UrdfSpawnConfig {
                base_body_type: RigidBodyType::Fixed,
                self_collisions: false,
                mesh_assets_root: asset_root.map(Path::to_path_buf),
                use_declared_inertial_masses: project.use_declared_inertias,
                ..UrdfSpawnConfig::default()
            },
        )?;
        world.entity_mut(spawned.robot).insert(Transform3 {
            rotation: rne_urdf_import::rpy_to_quat(Vec3::from_array(
                project.robot_rotation_rpy_rad,
            )),
            ..Transform3::IDENTITY
        });
        attach_urdf_document_articulation(
            &mut world,
            &document,
            &spawned,
            UrdfArticulationConfig {
                base_body_type: RigidBodyType::Fixed,
                multibody: true,
                use_joint_origin_rpy: true,
                weld_fixed_children: true,
                ..UrdfArticulationConfig::default()
            },
        )?;
        let mut joints = Vec::new();
        for info in project.joint_catalog()? {
            let joint = document
                .robot
                .joints
                .iter()
                .find(|joint| joint.name == info.name)
                .context("joint disappeared from robot document")?;
            let entity = spawned.links[&joint.child];
            let mut motor = world
                .get_mut::<JointMotor>(entity)
                .context("missing joint motor")?;
            motor.stiffness = 200.0;
            motor.gain = 28.0;
            motor.max_force = info.max_effort;
            motor.target_position = 0.0;
            motor.velocity_rad_s = 0.0;
            joints.push((info, entity));
        }
        for object in &project.objects {
            let entity = spawn_named(&mut world, format!("workbench/{}", object.name));
            world.entity_mut(entity).insert((
                Transform3 {
                    translation: Vec3::from_array(object.position_m),
                    ..Transform3::IDENTITY
                },
                RigidBody {
                    body_type: RigidBodyType::Fixed,
                    ..RigidBody::default()
                },
                Collider::cuboid(Vec3::from_array(object.size_m) * 0.5),
                Visual::new(
                    VisualShape::Box {
                        size_m: Vec3::from_array(object.size_m),
                    },
                    object.color_rgba,
                ),
            ));
        }
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc {
            solver_iterations: 12,
            ..PhysicsWorldDesc::default()
        })?;
        backend.sync_from_ecs(&mut world, physics_world)?;
        let mut robot_links = spawned.links.values().copied().collect::<Vec<_>>();
        robot_links.sort_by_key(|entity| entity.index());
        Ok(Self {
            project,
            world,
            backend,
            physics_world,
            clock: SimClock::new(SimDuration::from_ticks(STEP_TICKS)),
            joints,
            robot_links,
        })
    }

    pub fn camera_fit(&self) -> (Vec3, f64) {
        let mut minimum = Vec3::splat(f64::INFINITY);
        let mut maximum = Vec3::splat(f64::NEG_INFINITY);
        for entity in &self.robot_links {
            let point = world_transform_of(&self.world, *entity).translation;
            minimum = minimum.min(point);
            maximum = maximum.max(point);
        }
        let focus = (minimum + maximum) * 0.5;
        let distance_m = ((maximum - minimum).length() * 1.6 + 0.1).clamp(0.4, 30.0);
        (focus, distance_m)
    }

    pub fn set_targets(&mut self, targets: BTreeMap<String, JointPosition>) -> Result<()> {
        let mut candidate = self.project.clone();
        candidate.targets = targets;
        candidate.validate()?;
        self.project = candidate;
        Ok(())
    }

    pub fn step(&mut self, steps: u32) -> Result<()> {
        ensure!(
            (1..=240).contains(&steps),
            "step count must be between 1 and 240"
        );
        let dt = self.clock.fixed_delta();
        for _ in 0..steps {
            for (info, entity) in &self.joints {
                let goal = self.project.targets[&info.name].value();
                let mut motor = self
                    .world
                    .get_mut::<JointMotor>(*entity)
                    .context("missing servo")?;
                let delta = info.max_velocity * dt.as_seconds().value();
                motor.target_position += (goal - motor.target_position).clamp(-delta, delta);
            }
            step_physics(&mut self.backend, &mut self.world, self.physics_world, dt)?;
            self.clock.advance(dt);
        }
        Ok(())
    }

    pub fn sim_time_ticks(&self) -> u64 {
        self.clock.sim_time().ticks()
    }

    pub fn state_hash(&self) -> u64 {
        hash_physics_state(&self.world)
    }

    pub fn readings(&self) -> Result<Vec<JointReading>> {
        self.joints
            .iter()
            .map(|(info, entity)| {
                let (measured, velocity) = self
                    .backend
                    .multibody_joint_state(self.physics_world, *entity)
                    .context("joint state unavailable")?;
                ensure!(
                    measured.is_finite() && velocity.is_finite(),
                    "non-finite joint measurement"
                );
                Ok(JointReading {
                    info: info.clone(),
                    measured,
                    velocity,
                    target: self.project.targets[&info.name],
                })
            })
            .collect()
    }

    pub fn lidar(&self) -> Result<LidarReading> {
        let origin_m = [0.0, 0.5, 0.0];
        let max_range_m = 10.0;
        let mut ranges_m = Vec::with_capacity(180);
        let mut points_m = Vec::new();
        for column in 0..180 {
            let angle = std::f64::consts::TAU * f64::from(column) / 180.0;
            let hits = self.backend.raycast(
                self.physics_world,
                RaycastQuery {
                    origin_m: Vec3::from_array(origin_m),
                    direction: Vec3::new(angle.cos(), 0.0, angle.sin()),
                    max_distance_m: max_range_m,
                },
            )?;
            ranges_m.push(hits.first().map(|hit| hit.distance_m));
            if let Some(hit) = hits.first() {
                points_m.push(hit.point_m.to_array());
            }
        }
        Ok(LidarReading {
            sim_time_ticks: self.sim_time_ticks(),
            origin_m,
            max_range_m,
            ranges_m,
            points_m,
        })
    }

    pub fn render_scene(&self) -> RenderScene {
        let mut scene = RenderScene::new();
        let mut entities = self
            .world
            .iter_entities()
            .map(|e| e.id())
            .collect::<Vec<_>>();
        entities.sort_by_key(|e| e.index());
        for entity in entities {
            let transform = world_transform_of(&self.world, entity);
            if let Some(visuals) = self.world.get::<LinkVisuals>(entity) {
                for visual in &visuals.visuals {
                    scene.items.push(RenderScene::item_from_visual(
                        transform,
                        visual.shape.clone(),
                        visual.color_rgba,
                        visual.local_offset,
                    ));
                }
            } else if let Some(visual) = self.world.get::<Visual>(entity) {
                scene.items.push(RenderScene::item_from_visual(
                    transform,
                    visual.shape.clone(),
                    visual.color_rgba,
                    visual.local_offset,
                ));
            }
        }
        scene
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::{RobotFormat, RobotSource, SceneObject};

    fn slider() -> Project {
        Project::new(RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
                .into(),
        })
        .unwrap()
    }

    #[test]
    fn native_servo_moves_and_replay_is_deterministic() {
        let mut project = slider();
        project.targets.insert(
            "slider_joint".into(),
            JointPosition::Prismatic { position_m: 0.1 },
        );
        let mut first = Simulation::new(project.clone(), None).unwrap();
        let mut second = Simulation::new(project, None).unwrap();
        first.step(240).unwrap();
        second.step(240).unwrap();
        assert!(first.readings().unwrap()[0].measured > 0.06);
        assert_eq!(first.sim_time_ticks(), STEP_TICKS * 240);
        assert_eq!(first.state_hash(), second.state_hash());
    }

    #[test]
    fn scene_obstacle_changes_real_raycast_scan() {
        let project = slider();
        let empty = Simulation::new(project.clone(), None)
            .unwrap()
            .lidar()
            .unwrap();
        assert_eq!(empty.ranges_m[0], None);
        let mut with_box = project;
        with_box.objects.push(SceneObject {
            name: "wall".into(),
            position_m: [2.0, 0.5, 0.0],
            size_m: [0.2, 1.0, 1.0],
            color_rgba: [0.8, 0.2, 0.1, 1.0],
        });
        let scan = Simulation::new(with_box, None).unwrap().lidar().unwrap();
        assert!((scan.ranges_m[0].unwrap() - 1.9).abs() < 1e-5);
        assert_eq!(scan.sim_time_ticks, 0);
    }
}
