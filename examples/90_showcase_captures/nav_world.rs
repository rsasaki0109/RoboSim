//! The other agents in the navigation showcase and how the AGV perceives them.
//!
//! Three things share the corridor with the AGV: a second AGV driving the other
//! way, a pedestrian crossing, and a hand truck someone left in the aisle. Each
//! is a kinematic body in the physics world, so the AGV's LiDAR hits it. The
//! AGV knows none of them in advance: it segments its scan, drops the returns
//! that the static map explains, tracks what is left, and plans and avoids
//! against the tracks.

use anyhow::{Context, Result};
use rne_ai::DiffDriveSim;
use rne_ecs::{spawn_named, Entity};
use rne_math::{Quat, Vec3};
use rne_nav::{
    avoid_velocities, pure_pursuit_follow, AvoidanceConfig, CircularObstacle, Detection, Path2d,
    Pose2d, PurePursuitConfig, VelocityCommand2d,
};
use rne_physics::{Collider, ColliderShape, CommandedKinematicPose, RigidBody, RigidBodyType};
use rne_render::{load_gltf_scene, GltfAnimationPlayer, GltfSceneAsset, RenderScene, VisualShape};
use rne_sensor::{Sensor, SensorKind};
use rne_world::Transform3;
use std::path::Path;
use std::sync::Arc;

/// Planar half extents of either AGV: along its heading, across it.
pub(crate) const AGV_HALF_M: (f64, f64) = (0.25, 0.2);

/// The second AGV's lane and speed. It keeps right, which still leaves part of
/// it in the ego's straight path down the centre line.
const ONCOMING_START: (f64, f64) = (5.9, -0.45);
const ONCOMING_END: (f64, f64) = (-1.8, -0.45);
const ONCOMING_SPEED_M_S: f64 = 0.42;
const ONCOMING_DEPARTURE_S: f64 = 0.0;

/// The pedestrian walks across the corridor from the far wall's doorway.
pub(crate) const DOORWAY_X_M: f64 = 4.4;
const PEDESTRIAN_START_Z_M: f64 = -1.35;
const PEDESTRIAN_END_Z_M: f64 = 2.2;
const PEDESTRIAN_SPEED_M_S: f64 = 0.9;
const PEDESTRIAN_DEPARTURE_S: f64 = 6.0;
/// Radius of the circle bounding the pedestrian's body.
pub(crate) const PEDESTRIAN_RADIUS_M: f64 = 0.26;
/// Height the Rigged Figure is scaled to.
const PEDESTRIAN_HEIGHT_M: f64 = 1.7;

/// Collision half extents of the pedestrian and the hand truck.
const PEDESTRIAN_BODY_HALF_M: Vec3 = Vec3::new(0.2, 0.85, 0.16);
const HAND_TRUCK_BODY_HALF_M: Vec3 = Vec3::new(0.22, 0.55, 0.28);

/// The hand truck left against the near side of the aisle.
pub(crate) const HAND_TRUCK: (f64, f64, f64) = (5.35, 0.62, 0.35);

/// Half height of the second AGV's collision box, floor to the top of the tote
/// stack it carries. The ego's scan plane is 0.65 m up; the AGV's chassis alone
/// is lower, and the scan passed straight over it until the totes were added.
const AGV_BODY_HALF_Y_M: f64 = 0.4;
/// The tote stack on the second AGV's deck: size, and centre height.
pub(crate) const TOTES_SIZE_M: Vec3 = Vec3::new(0.46, 0.44, 0.36);
pub(crate) const TOTES_CENTER_Y_M: f64 = 0.6;

/// A body the corridor's other agents move by pose.
struct Kinematic {
    entity: Entity,
}

impl Kinematic {
    fn spawn(sim: &mut DiffDriveSim, name: &str, half: Vec3, at: Vec3) -> Self {
        let world = sim.world_mut();
        let entity = spawn_named(world, name);
        world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            CommandedKinematicPose,
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: half,
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(at, Quat::IDENTITY),
        ));
        Self { entity }
    }

    fn place(&self, sim: &mut DiffDriveSim, at: Vec3, rotation: Quat) {
        if let Some(mut transform) = sim.world_mut().get_mut::<Transform3>(self.entity) {
            transform.translation = at;
            transform.rotation = rotation;
        }
    }
}

/// A planar pose `(x, z, heading)` with heading measured in the (x, z) plane,
/// the frame the planner works in. The simulator's yaw is its negative.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PlanarPose {
    pub x_m: f64,
    pub z_m: f64,
    pub heading_rad: f64,
}

impl PlanarPose {
    pub fn sim_yaw_rad(self) -> f64 {
        -self.heading_rad
    }
}

/// The second AGV: it follows its own lane with pure pursuit and runs the same
/// reciprocal avoidance as the ego, fed the ego's pose over the fleet link.
pub(crate) struct OncomingAgv {
    body: Kinematic,
    lane: Path2d,
    pub pose: PlanarPose,
    pub velocity: VelocityCommand2d,
    /// Steps on which avoidance changed its command.
    pub yielded_steps: u32,
}

/// The pedestrian: a straight crossing at walking pace, and the animation
/// clock of the figure drawn for it.
pub(crate) struct Pedestrian {
    body: Kinematic,
    pub x_m: f64,
    pub z_m: f64,
    pub walking: bool,
    player: GltfAnimationPlayer,
}

/// Everything in the corridor besides the ego.
pub(crate) struct Corridor {
    pub oncoming: OncomingAgv,
    pub pedestrian: Pedestrian,
    figure: Arc<GltfSceneAsset>,
    figure_scale: f64,
}

impl Corridor {
    pub fn spawn(sim: &mut DiffDriveSim, repo_root: &Path) -> Result<Self> {
        let oncoming = OncomingAgv {
            body: Kinematic::spawn(
                sim,
                "oncoming_agv",
                Vec3::new(AGV_HALF_M.0, AGV_BODY_HALF_Y_M, AGV_HALF_M.1),
                Vec3::new(ONCOMING_START.0, AGV_BODY_HALF_Y_M + 0.02, ONCOMING_START.1),
            ),
            lane: Path2d::from_points(&[
                Vec3::new(ONCOMING_START.0, ONCOMING_START.1, 0.0),
                Vec3::new(ONCOMING_END.0, ONCOMING_END.1, 0.0),
            ]),
            pose: PlanarPose {
                x_m: ONCOMING_START.0,
                z_m: ONCOMING_START.1,
                heading_rad: std::f64::consts::PI,
            },
            velocity: VelocityCommand2d::ZERO,
            yielded_steps: 0,
        };
        let pedestrian = Pedestrian {
            body: Kinematic::spawn(
                sim,
                "pedestrian",
                PEDESTRIAN_BODY_HALF_M,
                Vec3::new(DOORWAY_X_M, 0.85, PEDESTRIAN_START_Z_M),
            ),
            x_m: DOORWAY_X_M,
            z_m: PEDESTRIAN_START_Z_M,
            walking: false,
            player: GltfAnimationPlayer::new(Some(0)),
        };
        let (x, z, heading) = HAND_TRUCK;
        let truck = Kinematic::spawn(
            sim,
            "hand_truck",
            HAND_TRUCK_BODY_HALF_M,
            Vec3::new(x, 0.55, z),
        );
        truck.place(sim, Vec3::new(x, 0.55, z), Quat::from_rotation_y(-heading));
        let figure =
            load_gltf_scene(&repo_root.join("assets/fixtures/rigged_figure/RiggedFigure.glb"))
                .context("load the Rigged Figure pedestrian")?;
        let figure_scale = PEDESTRIAN_HEIGHT_M / figure_height_m(&figure)?;
        let corridor = Self {
            oncoming,
            pedestrian,
            figure: Arc::new(figure),
            figure_scale,
        };
        corridor.place(sim);
        Ok(corridor)
    }

    /// Advances the pedestrian and the second AGV by `dt_s` at `time_s`.
    /// `ego` is the ego's pose and velocity as the fleet link reports them.
    pub fn advance(
        &mut self,
        sim: &mut DiffDriveSim,
        time_s: f64,
        dt_s: f64,
        ego: (PlanarPose, VelocityCommand2d),
    ) {
        let walker = &mut self.pedestrian;
        walker.walking = time_s >= PEDESTRIAN_DEPARTURE_S && walker.z_m < PEDESTRIAN_END_Z_M;
        if walker.walking {
            walker.z_m = (walker.z_m + PEDESTRIAN_SPEED_M_S * dt_s).min(PEDESTRIAN_END_Z_M);
            walker.player.advance(dt_s as f32);
        }
        if time_s >= ONCOMING_DEPARTURE_S {
            let walker = CircularObstacle {
                center_m: Vec3::new(self.pedestrian.x_m, self.pedestrian.z_m, 0.0),
                velocity_m_s: if self.pedestrian.walking {
                    Vec3::new(0.0, PEDESTRIAN_SPEED_M_S, 0.0)
                } else {
                    Vec3::ZERO
                },
                radius_m: PEDESTRIAN_RADIUS_M,
            };
            self.oncoming.drive(ego, walker, dt_s);
        }
        self.place(sim);
    }

    fn place(&self, sim: &mut DiffDriveSim) {
        let agv = self.oncoming.pose;
        self.oncoming.body.place(
            sim,
            Vec3::new(agv.x_m, AGV_BODY_HALF_Y_M + 0.02, agv.z_m),
            Quat::from_rotation_y(agv.sim_yaw_rad()),
        );
        self.pedestrian.body.place(
            sim,
            Vec3::new(self.pedestrian.x_m, 0.85, self.pedestrian.z_m),
            Quat::IDENTITY,
        );
    }

    /// Stands the Z-up figure on the floor: -z (head) to +y, and its +y (front) to
    /// +z, the way the pedestrian walks.
    const FIGURE_UPRIGHT: Quat = Quat::from_xyzw(
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
    );

    /// Removes the kinematic bodies' collision boxes from a scene built from the
    /// physics world, which draws a collider wherever a body has no visual. The
    /// agents are drawn from their own models instead.
    pub(crate) fn hide_bodies(scene: &mut RenderScene) {
        let sizes = [
            Vec3::new(
                2.0 * AGV_HALF_M.0,
                2.0 * AGV_BODY_HALF_Y_M,
                2.0 * AGV_HALF_M.1,
            ),
            PEDESTRIAN_BODY_HALF_M * 2.0,
            HAND_TRUCK_BODY_HALF_M * 2.0,
        ];
        scene.items.retain(|item| {
            !matches!(item.shape, VisualShape::Box { size_m }
            if sizes.iter().any(|size| (*size - size_m).length() < 1e-9))
        });
    }

    /// Draws the pedestrian as the walking Rigged Figure, facing +z.
    pub fn push_pedestrian(&self, scene: &mut RenderScene) -> Result<()> {
        let walker = &self.pedestrian;
        for part_index in 0..self.figure.parts.len() {
            let part = &self.figure.parts[part_index];
            // Deformed on the CPU: the sample carries the figure's node
            // transform and skin already, so the item transform only places
            // and scales it.
            let mesh = walker
                .player
                .sample_part_cpu(&self.figure, part_index)
                .context("sample the pedestrian")?;
            let mut item = RenderScene::item_from_dynamic_mesh(mesh, [1.0; 4]);
            item.transform.translation = Vec3::new(walker.x_m, 0.0, walker.z_m);
            item.transform.rotation = Self::FIGURE_UPRIGHT;
            item.transform.scale = Vec3::splat(self.figure_scale);
            item.base_color_texture = part.render_part.base_color_texture.clone().map(Arc::new);
            item.material = part.render_part.material.clone();
            // The figure ships untextured white, which disappears against the
            // pale floor; dressed in a dark work jacket it reads as a person.
            item.color_rgba = [1.0; 4];
            item.material.base_color_rgba = [0.22, 0.30, 0.46, 1.0];
            item.material.roughness = 0.8;
            scene.items.push(item);
        }
        Ok(())
    }
}

/// Height of the figure's bind pose. The Rigged Figure is modelled Z-up with its
/// feet at the origin and its head toward -z; [`FIGURE_UPRIGHT`] stands it up.
fn figure_height_m(figure: &GltfSceneAsset) -> Result<f64> {
    let player = GltfAnimationPlayer::new(Some(0));
    let (mut low, mut high) = (f32::INFINITY, f32::NEG_INFINITY);
    for part_index in 0..figure.parts.len() {
        let mesh = player
            .sample_part_cpu(figure, part_index)
            .context("sample the pedestrian bind pose")?;
        for position in &mesh.positions {
            low = low.min(position[2]);
            high = high.max(position[2]);
        }
    }
    anyhow::ensure!(high > low, "the pedestrian figure has no height");
    Ok(f64::from(high - low))
}

impl OncomingAgv {
    /// One step of the second AGV. It has no simulated sensors: it is given
    /// the ego's pose over the fleet link and the pedestrian's true position.
    fn drive(&mut self, ego: (PlanarPose, VelocityCommand2d), walker: CircularObstacle, dt_s: f64) {
        let pursuit = PurePursuitConfig {
            lookahead_m: 0.8,
            max_linear_m_s: ONCOMING_SPEED_M_S,
            max_angular_rad_s: 1.2,
            goal_tolerance_m: 0.1,
            slow_radius_m: 0.3,
        };
        let pose = Pose2d::new(self.pose.x_m, self.pose.z_m, self.pose.heading_rad);
        let desired = pure_pursuit_follow(&self.lane, pose, &pursuit)
            .map(|follow| follow.command)
            .unwrap_or(VelocityCommand2d::ZERO);
        let (ego_pose, ego_velocity) = ego;
        let ego_obstacle = CircularObstacle {
            center_m: Vec3::new(ego_pose.x_m, ego_pose.z_m, 0.0),
            velocity_m_s: Vec3::new(
                ego_velocity.linear_m_s * ego_pose.heading_rad.cos(),
                ego_velocity.linear_m_s * ego_pose.heading_rad.sin(),
                0.0,
            ),
            radius_m: AGV_RADIUS_M,
        };
        let command = avoid_velocities(
            pose,
            AGV_RADIUS_M,
            &[ego_obstacle, walker],
            desired,
            ONCOMING_SPEED_M_S,
            1.2,
            &AvoidanceConfig {
                // Shorter than the ego's: the second AGV holds its lane and
                // brakes only when contact is close.
                time_horizon_s: 1.0,
                ..avoidance_config()
            },
        );
        if (command.linear_m_s - desired.linear_m_s).abs() > 0.05
            || (command.angular_rad_s - desired.angular_rad_s).abs() > 0.1
        {
            self.yielded_steps += 1;
        }
        self.velocity = command;
        self.pose.heading_rad += command.angular_rad_s * dt_s;
        self.pose.x_m += command.linear_m_s * self.pose.heading_rad.cos() * dt_s;
        self.pose.z_m += command.linear_m_s * self.pose.heading_rad.sin() * dt_s;
    }
}

/// Radius of the circle that bounds either AGV's footprint.
pub(crate) const AGV_RADIUS_M: f64 = 0.33;

/// Local avoidance settings shared by both AGVs.
pub(crate) fn avoidance_config() -> AvoidanceConfig {
    AvoidanceConfig {
        time_horizon_s: 1.6,
        simulation_step_s: 0.1,
        linear_samples: 11,
        angular_samples: 21,
        safety_margin_m: 0.05,
    }
}

/// Raises the ego LiDAR to one ray per degree. The office AGV's asset ships a
/// 90-ray (4 degree) scan, which puts a pedestrian three meters away between
/// two rays.
pub(crate) fn densify_lidar(sim: &mut DiffDriveSim) -> Result<()> {
    let lidar = sim
        .lidar_mounts()
        .first()
        .context("office AGV has no LiDAR")?
        .lidar;
    let mut sensor = sim
        .world_mut()
        .get_mut::<Sensor>(lidar)
        .context("LiDAR sensor component")?;
    match &mut sensor.kind {
        SensorKind::Lidar(spec) => {
            spec.ray_count = 360;
            Ok(())
        }
        _ => anyhow::bail!("primary LiDAR is not a LiDAR"),
    }
}

/// Splits a scan into obstacles the static map does not explain.
///
/// Returns in ray order; a gap of more than `SEGMENT_GAP_M` between consecutive
/// returns starts a new segment. Each segment becomes one detection, centred on
/// its returns with a radius that covers them.
pub(crate) fn detect(points: &[Vec3], explained: impl Fn(f64, f64) -> bool) -> Vec<Detection> {
    const SEGMENT_GAP_M: f64 = 0.3;
    let mut detections = Vec::new();
    let mut segment: Vec<(f64, f64)> = Vec::new();
    let mut flush = |segment: &mut Vec<(f64, f64)>| {
        if segment.len() >= 2 {
            let n = segment.len() as f64;
            let cx = segment.iter().map(|p| p.0).sum::<f64>() / n;
            let cz = segment.iter().map(|p| p.1).sum::<f64>() / n;
            let reach = segment
                .iter()
                .map(|p| (p.0 - cx).hypot(p.1 - cz))
                .fold(0.0, f64::max);
            detections.push(Detection {
                position_m: Vec3::new(cx, cz, 0.0),
                radius_m: (reach + 0.08).max(0.18),
            });
        }
        segment.clear();
    };
    for point in points {
        if explained(point.x, point.z) {
            flush(&mut segment);
            continue;
        }
        if let Some(last) = segment.last() {
            if (point.x - last.0).hypot(point.z - last.1) > SEGMENT_GAP_M {
                flush(&mut segment);
            }
        }
        segment.push((point.x, point.z));
    }
    flush(&mut segment);
    detections
}

/// Renders the hand truck prop where its body stands.
pub(crate) fn push_hand_truck(scene: &mut RenderScene) {
    let (x, z, heading) = HAND_TRUCK;
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(
            Vec3::new(x, 0.0, z),
            Quat::from_rotation_y(-heading),
        ),
        VisualShape::Mesh {
            path: "hand_truck/hand_truck_1k.gltf".to_string(),
            scale: Vec3::ONE,
        },
        [1.0; 4],
        Transform3::IDENTITY,
    ));
}

/// Draws the tote stack on the second AGV's deck: two plastic totes with lids.
pub(crate) fn push_totes(scene: &mut RenderScene, pose: PlanarPose) {
    let rotation = Quat::from_rotation_y(pose.sim_yaw_rad());
    let base = Vec3::new(pose.x_m, TOTES_CENTER_Y_M - 0.5 * TOTES_SIZE_M.y, pose.z_m);
    for (level, color) in [
        (0.0, [0.12, 0.42, 0.78, 1.0]),
        (1.0, [0.95, 0.60, 0.10, 1.0]),
    ] {
        let height = 0.5 * TOTES_SIZE_M.y;
        let centre = base + Vec3::new(0.0, height * (level + 0.5), 0.0);
        let mut tote = RenderScene::item_from_visual(
            Transform3::from_translation_rotation(centre, rotation),
            VisualShape::Box {
                size_m: Vec3::new(TOTES_SIZE_M.x, height - 0.01, TOTES_SIZE_M.z),
            },
            color,
            Transform3::IDENTITY,
        );
        tote.material = rne_render::PbrMaterial::new(color, 0.55, 0.0, [0.0; 3]);
        scene.items.push(tote);
        let mut rim = RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                centre + Vec3::new(0.0, 0.5 * height - 0.01, 0.0),
                rotation,
            ),
            VisualShape::Box {
                size_m: Vec3::new(TOTES_SIZE_M.x + 0.02, 0.02, TOTES_SIZE_M.z + 0.02),
            },
            [0.2, 0.2, 0.22, 1.0],
            Transform3::IDENTITY,
        );
        rim.material = rne_render::PbrMaterial::new([0.2, 0.2, 0.22, 1.0], 0.5, 0.0, [0.0; 3]);
        scene.items.push(rim);
    }
}
