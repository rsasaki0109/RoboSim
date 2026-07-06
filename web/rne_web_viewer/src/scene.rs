//! Embedded `mm_minimal` scene loading and kinematic pose updates for the web viewer.

use rne_assets::{
    parse_scene_bundle_with_sources, spawn_scene_bundle, SpawnSceneOptions, SpawnedScene,
    UrdfSourceMap,
};
use rne_ecs::{Entity, Name, World};
use rne_math::{Quat, Vec3};
use rne_physics::Collider;
use rne_render::{RenderScene, Visual, VisualShape};
use rne_robot::Joint;
use rne_urdf_import::{parse_urdf, rpy_to_quat, UrdfJointType};
use rne_world::{world_transform_of, Transform3 as WorldTransform3};
use std::path::PathBuf;

const SCENE_VIRTUAL_PATH: &str = "assets/scenes/mm_minimal.rne.scene.toml";
const ROBOT_VIRTUAL_PATH: &str = "assets/robots/mm_minimal.rne.robot.toml";
const URDF_VIRTUAL_PATH: &str = "assets/robots/mm_minimal/mm_minimal.urdf";

const SCENE_TOML: &str = include_str!("../../../assets/scenes/mm_minimal.rne.scene.toml");
const ROBOT_TOML: &str = include_str!("../../../assets/robots/mm_minimal.rne.robot.toml");
const URDF_XML: &str = include_str!("../../../assets/robots/mm_minimal/mm_minimal.urdf");

const BASE_COLOR: [f32; 4] = [0.35, 0.55, 0.95, 1.0];

/// One revolute joint driven by the deterministic web animation.
#[derive(Clone, Debug)]
struct JointDrive {
    joint_entity: Entity,
    child_link: Entity,
    origin_xyz: Vec3,
    origin_rpy: Vec3,
    axis: Vec3,
    amplitude_rad: f64,
    phase_rad: f64,
}

/// Loaded `mm_minimal` scene ready for kinematic updates and rendering.
#[derive(Debug)]
pub struct WebScene {
    world: World,
    joint_drives: Vec<JointDrive>,
    focus: Vec3,
}

impl WebScene {
    /// Loads the embedded `mm_minimal` scene without filesystem access.
    pub fn load_mm_minimal() -> Result<Self, String> {
        let scene_path = PathBuf::from(SCENE_VIRTUAL_PATH);
        let robot_path = PathBuf::from(ROBOT_VIRTUAL_PATH);
        let urdf_path = PathBuf::from(URDF_VIRTUAL_PATH);

        let mut urdf_sources = UrdfSourceMap::new();
        urdf_sources.insert(urdf_path, URDF_XML);

        let bundle = parse_scene_bundle_with_sources(
            &scene_path,
            SCENE_TOML,
            &[(robot_path, ROBOT_TOML)],
            Some(&urdf_sources),
        )
        .map_err(|error| error.to_string())?;

        let mut world = World::new();
        let spawned = spawn_scene_bundle(
            &mut world,
            &bundle,
            Some(&urdf_sources),
            SpawnSceneOptions {
                wire_articulation: false,
            },
        )
        .map_err(|error| error.to_string())?;

        let joint_drives = joint_drives_from_spawn(&world)?;
        let focus = scene_focus(&world, &spawned);

        Ok(Self {
            world,
            joint_drives,
            focus,
        })
    }

    /// Orbit-camera focus point in world space.
    pub fn focus(&self) -> Vec3 {
        self.focus
    }

    /// Advances the deterministic joint sweep and returns a render scene.
    ///
    /// `frame_index` is a monotonic counter incremented once per `requestAnimationFrame`
    /// callback. Animation uses only this counter (not wall-clock time) so the sweep
    /// stays reproducible and does not violate SimClock rules for simulation logic.
    pub fn frame(&mut self, frame_index: u64) -> RenderScene {
        self.apply_joint_animation(frame_index);
        build_visual_render_scene(&self.world)
    }

    fn apply_joint_animation(&mut self, frame_index: u64) {
        const PERIOD_FRAMES: u64 = 480;
        let phase = (frame_index % PERIOD_FRAMES) as f64 / PERIOD_FRAMES as f64
            * std::f64::consts::TAU;

        for drive in &self.joint_drives {
            let angle = drive.amplitude_rad * (phase + drive.phase_rad).sin();
            apply_revolute_joint(&mut self.world, drive, angle);
        }
    }
}

fn joint_drives_from_spawn(world: &World) -> Result<Vec<JointDrive>, String> {
    let urdf = parse_urdf(URDF_XML).map_err(|error| error.to_string())?;
    let mut drives = Vec::new();
    for joint in &urdf.joints {
        match joint.joint_type {
            UrdfJointType::Revolute | UrdfJointType::Continuous => {}
            _ => continue,
        }

        let joint_entity = find_joint_entity(world, &joint.name)?;
        let joint_comp = world
            .get::<Joint>(joint_entity)
            .ok_or_else(|| format!("missing joint component for `{}`", joint.name))?;

        let (amplitude_rad, phase_rad) = animation_profile(&joint.name);
        drives.push(JointDrive {
            joint_entity,
            child_link: joint_comp.child_link,
            origin_xyz: joint.origin_xyz,
            origin_rpy: joint.origin_rpy,
            axis: joint.axis,
            amplitude_rad,
            phase_rad,
        });
    }

    if drives.is_empty() {
        return Err("no actuated joints found".into());
    }

    Ok(drives)
}

fn find_joint_entity(world: &World, joint_name: &str) -> Result<Entity, String> {
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world
            .get::<Name>(entity)
            .is_some_and(|name| name.0 == joint_name)
        {
            return Ok(entity);
        }
    }
    Err(format!("joint entity `{joint_name}` not found"))
}

fn animation_profile(joint_name: &str) -> (f64, f64) {
    match joint_name {
        "shoulder_joint" => (0.75, 0.0),
        "elbow_joint" => (0.55, 0.9),
        "left_finger_joint" => (0.12, 1.8),
        "right_finger_joint" => (-0.12, 1.8),
        _ => (0.25, 0.0),
    }
}

fn apply_revolute_joint(world: &mut World, drive: &JointDrive, position_rad: f64) {
    let axis = drive.axis.normalize_or_zero();
    let joint_rotation = if axis.length_squared() > f64::EPSILON {
        Quat::from_axis_angle(axis, position_rad)
    } else {
        Quat::IDENTITY
    };
    let origin_rotation = rpy_to_quat(drive.origin_rpy);

    if let Some(mut transform) = world.get_mut::<WorldTransform3>(drive.child_link) {
        *transform = WorldTransform3::from_translation_rotation(
            drive.origin_xyz,
            origin_rotation * joint_rotation,
        );
    }
    if let Some(mut joint) = world.get_mut::<Joint>(drive.joint_entity) {
        joint.position = position_rad;
        joint.velocity = 0.0;
    }
}

fn scene_focus(world: &World, spawned: &SpawnedScene) -> Vec3 {
    let (_, robot) = spawned
        .robots
        .first()
        .expect("spawned robot present after load");
    let base = world_transform_of(world, robot.base_link);
    Vec3::new(base.translation.x, base.translation.y + 0.35, base.translation.z)
}

/// Builds a render scene from all entities that carry visuals or colliders.
pub fn build_visual_render_scene(world: &World) -> RenderScene {
    let mut scene = RenderScene::new();
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world.get::<Visual>(entity).is_some() || world.get::<Collider>(entity).is_some() {
            append_entity_visual(&mut scene, world, entity);
        }
    }
    scene
}

fn append_entity_visual(scene: &mut RenderScene, world: &World, entity: Entity) {
    let world_transform = world_transform_of(world, entity);

    if let Some(visual) = world.get::<Visual>(entity) {
        scene.items.push(RenderScene::item_from_visual(
            world_transform,
            visual.shape.clone(),
            visual.color_rgba,
            visual.local_offset,
        ));
        return;
    }

    if let Some(collider) = world.get::<Collider>(entity) {
        if let Some((shape, color)) = collider_fallback_visual(collider) {
            scene.items.push(RenderScene::item_from_visual(
                world_transform,
                shape,
                color,
                WorldTransform3::IDENTITY,
            ));
        }
    }
}

fn collider_fallback_visual(collider: &Collider) -> Option<(VisualShape, [f32; 4])> {
    match collider.shape {
        rne_physics::ColliderShape::Cuboid { half_extents_m } => Some((
            VisualShape::Box {
                size_m: half_extents_m * 2.0,
            },
            BASE_COLOR,
        )),
        rne_physics::ColliderShape::Sphere { radius_m } => {
            Some((VisualShape::Sphere { radius_m }, BASE_COLOR))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_mm_minimal_loads() {
        let mut scene = WebScene::load_mm_minimal().expect("load embedded scene");
        let frame_a = scene.frame(0);
        let frame_b = scene.frame(120);
        assert!(frame_a.items.len() >= 6, "expected arm links + ground");
        assert_eq!(frame_a.items.len(), frame_b.items.len());
    }
}
