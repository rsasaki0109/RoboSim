//! `LiDAR` sampling on a URDF scene that sees the environment but not the robot.
//!
//! A robot's own body shows up in real scans, but for a mounted sensor those returns
//! are measured per bin in a [`LidarRigOcclusion`] table, which also carries the
//! mount posts the collision model does not have. Casting against the robot's
//! colliders as well would count the body twice, so the scene's raycasts skip the
//! links of the robot carrying the sensor. Links of other articulated bodies in the
//! scene, such as a door on its hinge, are scanned like any other surface.

use super::UrdfSceneSim;
use rne_data::PointCloud;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{PhysicsBackend, PhysicsError, PhysicsWorldId, RaycastHit, RaycastQuery};
use rne_physics_rapier::RapierBackend;
use rne_robot::Link;
use rne_sensor::{
    sample_livox_mid360, LidarRaycaster, LidarRigOcclusion, LidarSpec, LidarSweep,
    LivoxMid360Pattern, SensorNoiseKey,
};
use rne_world::Transform3;

/// Height of the Go2's upside-down Mid-360 above the base link origin, in meters.
///
/// The recordings put the floor 0.440–0.454 m from the sensor; the Go2 trot stands its
/// base 0.30 m above the floor, so the sensor sits 0.147 m above the base origin.
pub const UNITREE_GO2_MID360_HEIGHT_ABOVE_BASE_M: f64 = 0.147;
/// Forward offset of the Go2's Mid-360 from the base link origin, in meters.
///
/// Estimated, not measured: the recorded self returns extend 0.03–0.28 m behind the
/// sensor, which places it near the front edge of the 0.376 m body.
pub const UNITREE_GO2_MID360_FORWARD_OF_BASE_M: f64 = 0.20;

/// Mount of the Go2's Mid-360 relative to the `base` link, in URDF axes.
///
/// The sensor is upside down: Livox `x` looks forward and Livox `z` points at the
/// floor. In the engine's sensor convention (`+X` forward, `+Y` Livox up, `+Z` Livox
/// right) that is a −90° rotation about the base's forward axis.
pub fn unitree_go2_mid360_mount() -> Transform3 {
    Transform3::from_translation_rotation(
        Vec3::new(
            UNITREE_GO2_MID360_FORWARD_OF_BASE_M,
            0.0,
            UNITREE_GO2_MID360_HEIGHT_ABOVE_BASE_M,
        ),
        Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
    )
}

impl UrdfSceneSim {
    /// Returns the world pose of a frame rigidly attached to link `name`.
    pub fn named_mount_transform(&self, name: &str, mount: &Transform3) -> Option<Transform3> {
        self.named_transform(name)
            .map(|link| link.mul_transform(mount))
    }

    /// Samples one Livox Mid-360 frame against the scene without the robot's links.
    ///
    /// `sweep` is the sensor pose at the start and end of the frame, typically from
    /// [`Self::named_mount_transform`] before and after stepping one frame period.
    /// Everything else follows [`rne_sensor::sample_livox_mid360`].
    // Mirrors rne_sensor::sample_livox_mid360, whose inputs are independent.
    #[allow(clippy::too_many_arguments)]
    pub fn sample_livox_mid360(
        &self,
        sweep: &LidarSweep,
        spec: &LidarSpec,
        pattern: &LivoxMid360Pattern,
        frame_index: u64,
        rig: Option<&LidarRigOcclusion>,
        noise_key: SensorNoiseKey,
    ) -> PointCloud {
        let environment = EnvironmentRaycaster {
            backend: &self.backend,
            world: &self.world,
            own_robot: self
                .world
                .get::<Link>(self.base_link)
                .map(|link| link.robot),
        };
        sample_livox_mid360(
            &environment,
            self.physics_world,
            &self.world,
            sweep,
            spec,
            pattern,
            frame_index,
            rig,
            noise_key,
        )
    }
}

/// Read-only view of the scene backend whose raycasts skip the sensor robot's links.
struct EnvironmentRaycaster<'a> {
    backend: &'a RapierBackend,
    world: &'a World,
    /// The robot entity whose links the scan skips.
    own_robot: Option<rne_ecs::Entity>,
}

impl LidarRaycaster for EnvironmentRaycaster<'_> {
    fn lidar_raycast(
        &self,
        physics_world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let mut hits = self.backend.raycast(physics_world, query)?;
        hits.retain(|hit| {
            self.world
                .get::<Link>(hit.entity)
                .is_none_or(|link| Some(link.robot) != self.own_robot)
        });
        Ok(hits)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{unitree_go2_trot_targets, UnitreeGo2GaitCommand};
    use rne_sensor::livox_mid360_spec;
    use std::path::PathBuf;

    fn standing_go2() -> UrdfSceneSim {
        standing_go2_in("unitree_go2_jump.rne.scene.toml")
    }

    fn standing_go2_in(scene: &str) -> UrdfSceneSim {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes")
            .join(scene);
        let mut sim = UrdfSceneSim::from_scene_path(&path).expect("load welded Go2");
        sim.configure_position_motors(180.0, 18.0, 23.7);
        let stand = unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                ..UnitreeGo2GaitCommand::default()
            },
        );
        for _ in 0..240 {
            sim.step_joint_position_targets(&stand);
        }
        sim
    }

    fn go2_rig() -> LidarRigOcclusion {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/sensors/livox_mid360/go2_rig_occlusion.json");
        serde_json::from_str(&std::fs::read_to_string(path).expect("rig asset")).expect("rig")
    }

    #[test]
    fn standing_go2_mid360_sees_the_floor_and_not_itself() {
        let sim = standing_go2();
        let pose = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        // Livox +z (engine sensor +Y) points at the floor on the upside-down mount.
        let livox_up = pose.rotation * Vec3::Y;
        // Standing, the base rides 0.03 m higher than in the trot (recorded: floor
        // 0.440–0.454 m from the sensor); measured 0.475 m here.
        assert!(
            (0.44..0.50).contains(&pose.translation.y),
            "sensor height {:.3} m",
            pose.translation.y
        );
        assert!(
            livox_up.y < -0.999,
            "Livox +z must point at the floor: {livox_up:?}"
        );
        let spec = livox_mid360_spec();
        let rig = go2_rig();
        let pattern = LivoxMid360Pattern::new();
        let sweep = LidarSweep::stationary(pose);
        let mut slots = [0_usize; 7];
        let mut empty = [0_usize; 7];
        let mut floor_points = 0;
        let mut other_points = 0;
        for frame in 0..10 {
            let cloud = sim.sample_livox_mid360(
                &sweep,
                &spec,
                &pattern,
                frame,
                Some(&rig),
                SensorNoiseKey::new(sim.world_seed(), spec.seed, 1, frame),
            );
            let mut returned = std::collections::HashSet::new();
            for (index, point) in cloud.points_m.iter().enumerate() {
                returned.insert((cloud.ray_indices[index], cloud.channel_indices[index]));
                let range_m = (*point - pose.translation).length();
                if range_m >= 0.35 {
                    if point.y.abs() < 0.05 {
                        floor_points += 1;
                    } else {
                        other_points += 1;
                    }
                }
            }
            for ray in pattern.frame_rays(frame) {
                let band = ((ray.elevation_rad.to_degrees() - 24.0) / 4.0).floor();
                if !(0.0..7.0).contains(&band) {
                    continue;
                }
                slots[band as usize] += 1;
                if !returned.contains(&(ray.column, ray.channel)) {
                    empty[band as usize] += 1;
                }
            }
        }
        let bands = (0..7)
            .map(|band| empty[band] as f64 / slots[band] as f64)
            .collect::<Vec<_>>();
        // Only the floor is in the scene: every cast return lands on it and none on the
        // robot, whose returns come from the rig table instead.
        assert!(floor_points > 10_000, "floor returns {floor_points}");
        assert_eq!(other_points, 0, "cast returns off the floor");
        // Recorded EIL_Box no-return fraction per 4° band from 24°; measured here
        // 0.310, 0.312, 0.372, 0.488, 0.626, 0.762, 0.989 — the sensor stands higher
        // than while walking, so near-range loss is slightly lower.
        const RECORDED: [f64; 7] = [0.338, 0.343, 0.410, 0.533, 0.673, 0.772, 0.991];
        for (band, (model, recorded)) in bands.iter().zip(RECORDED).enumerate() {
            assert!(
                (model - recorded).abs() < 0.06,
                "band {}: {model:.3} vs recorded {recorded}",
                24 + 4 * band
            );
        }
    }

    #[test]
    fn mid360_scans_other_articulated_bodies_but_not_its_own_robot() {
        // The door is a URDF body of its own: its leaf must show in the scan, while
        // the Go2's links stay out (their returns come from the rig table).
        let sim = standing_go2_in("unitree_go2_door.rne.scene.toml");
        let pose = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let base = sim.named_transform("base").expect("base pose").translation;
        let spec = livox_mid360_spec();
        let pattern = LivoxMid360Pattern::new();
        let mut door_points = 0;
        let mut robot_points = 0;
        for frame in 0..5 {
            let cloud = sim.sample_livox_mid360(
                &LidarSweep::stationary(pose),
                &spec,
                &pattern,
                frame,
                None,
                SensorNoiseKey::new(sim.world_seed(), spec.seed, 1, frame),
            );
            for point in &cloud.points_m {
                // The closed leaf: x 2.455..2.485, z 0.21..1.17, up to 0.92 m high.
                if (2.44..2.50).contains(&point.x)
                    && (0.21..1.17).contains(&point.z)
                    && point.y < 0.93
                {
                    door_points += 1;
                }
                let local = *point - base;
                if local.x.abs() < 0.35 && local.z.abs() < 0.2 && point.y > 0.1 {
                    robot_points += 1;
                }
            }
        }
        assert!(door_points > 100, "door returns {door_points}");
        assert_eq!(robot_points, 0, "cast returns on the Go2 itself");
    }
}
