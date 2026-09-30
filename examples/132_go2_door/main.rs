//! The arm-carrying Go2 walks through a swing door and shuts it behind itself,
//! pushing the door only with the pad on its arm and steering on what its Mid-360
//! sees.
//!
//! * **Robot.** `unitree_go2_arm`: the welded Go2 with a generic 4-DOF arm (1.3 kg)
//!   on its back, on `rne_ai::UnitreeGo2ModelTrot` carrying the extra mass.
//! * **Door.** A 6 kg leaf on a damped hinge (`swing_door`), a separate articulated
//!   body in the two-room scene's doorway. Nothing holds, welds, or drives it: it
//!   moves only when something pushes it.
//! * **Opening.** The Go2 walks the doorway's centre line with the pad turned 0.35 m
//!   to its left, toward the hinge. The pad pushes the leaf near the hinge, so the
//!   leaf swings ahead of the body and the body never meets it.
//! * **Closing.** From the far side it walks around the open leaf and back along a
//!   line 0.52 m east of the hinge with the pad on its right, pushing the leaf
//!   nearly shut, then stops beside it and sweeps the pad outward to press the
//!   last degrees shut.
//! * **Localization.** As in example 131: Mid-360 returns in the sensor frame at
//!   emission time, levelled by IMU attitude and de-skewed by drifting leg odometry,
//!   feed `rne_slam::Slam2d`. Every command comes from that estimate; the door's
//!   position is given, as a map annotation would give it, and its angle is read
//!   only to score the run.
//!
//! The gate checks that the door opens past 80°, ends within 1° of shut, is
//! touched by the pad and by no other part of the robot, and that the robot stays
//! upright and localized.
//!
//! ```text
//! cargo run --release -p go2_door --example 132_go2_door -- --smoke
//! cargo run --release -p go2_door --example 132_go2_door -- --capture
//! ```

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, PI};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_mid360_mount, UnitreeGo2ModelTrot,
    UnitreeGo2TrotCommand, UrdfJointPositionTarget, UrdfSceneSim,
    UNITREE_GO2_MID360_FORWARD_OF_BASE_M, UNITREE_GO2_MODEL_TROT_CONTROL_HZ,
};
use rne_data::PointCloud;
use rne_math::{Quat, Transform3, Vec3};
use rne_nav::{FrameId, LaserScan2d, OccupancyGrid, Pose2d};
use rne_physics::CollisionGroups;
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_sensor::{
    livox_mid360_spec, LidarRigOcclusion, LidarSpec, LidarSweep, LivoxMid360Pattern, SensorNoiseKey,
};
use rne_slam::{Slam2d, SlamConfig};
use rne_world::Transform3 as WorldTransform3;
use sha2::{Digest, Sha256};

const MAX_DURATION_S: f64 = 180.0;
const STEPS_PER_LIDAR_FRAME: u64 = 50;
const KEYFRAME_TRANSLATION_M: f64 = 0.15;
const KEYFRAME_ROTATION_RAD: f64 = 0.15;
const ODOM_SCALE_ERROR: f64 = 0.04;
const ODOM_YAW_RATE_BIAS_RAD_S: f64 = 0.02;
const SCAN_MIN_HEIGHT_M: f64 = -0.30;
const SCAN_MAX_HEIGHT_M: f64 = 0.20;
const SCAN_MIN_RANGE_M: f64 = 0.40;
const SCAN_MAX_RANGE_M: f64 = 20.0;
const SCAN_BEAMS: usize = 720;
const MAP_ORIGIN: [f64; 2] = [-3.0, -3.0];
const MAP_SIZE_M: [f64; 2] = [10.5, 6.0];
const MAP_RESOLUTION_M: f64 = 0.05;

/// The Go2 with its 1.3 kg arm.
const TOTAL_MASS_KG: f64 = 16.1 + 1.3;
const ARM_JOINTS: [&str; 4] = ["arm_yaw", "arm_upper", "arm_fore", "arm_wrist"];
/// Arm effort limits from the URDF, N·m.
const ARM_EFFORT_NM: [f64; 4] = [10.0, 20.0, 12.0, 5.0];
/// Shoulder pitch axis in the base frame (x forward, z up), meters.
const SHOULDER_X_M: f64 = 0.05;
const SHOULDER_Z_M: f64 = 0.197;
const UPPER_M: f64 = 0.28;
const FORE_M: f64 = 0.28;
const PAD_FROM_WRIST_M: f64 = 0.095;
/// Folded over the back.
const STOW_POSE: [f64; 4] = [0.0, -1.3, 2.5, 0.3];

/// The door, in the navigation frame (x, y = -world z): hinge and doorway line.
const HINGE_NAV: [f64; 2] = [2.47, -0.2];
const DOORWAY_CENTRE_Y: f64 = -0.7;
/// Closing line: 0.52 m east of the hinge, so the pad 0.4 m to the right pushes
/// the leaf 0.12 m from the hinge line.
const CLOSING_LINE_X: f64 = HINGE_NAV[0] + 0.52;
const PUSH_SPEED_M_S: f64 = 0.10;
/// Final nudge: the pad's lateral offset sweeps from beside the body outward.
const NUDGE_START_LATERAL_M: f64 = -0.30;
const NUDGE_END_LATERAL_M: f64 = -0.62;
const NUDGE_SETTLE_S: f64 = 1.0;
const NUDGE_SWEEP_S: f64 = 2.0;
const CLOSE_SPEED_M_S: f64 = 0.12;

/// Robot links checked for contact with the door leaf.
const ROBOT_LINKS: [&str; 19] = [
    "base",
    "Head_upper",
    "arm_mount",
    "arm_yaw",
    "arm_upper",
    "arm_fore",
    "arm_wrist",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
];
/// The only link allowed to touch the door: it carries the push pad.
const PAD_LINK: &str = "arm_wrist";

/// Static walls and furniture in world `(centre [x, z], half extents [x, z])`.
const OBSTACLES: [([f64; 2], [f64; 2]); 11] = [
    ([-2.45, 0.0], [0.05, 2.5]),
    ([7.05, 0.0], [0.05, 2.5]),
    ([2.3, -2.45], [4.8, 0.05]),
    ([2.3, 2.45], [4.8, 0.05]),
    ([2.4, -1.1], [0.05, 1.3]),
    ([2.4, 1.8], [0.05, 0.6]),
    ([0.2, -1.3], [0.8, 0.45]),
    ([-1.5, 1.4], [0.3, 0.3]),
    ([4.3, -1.9], [0.8, 0.25]),
    ([5.0, 0.9], [0.25, 0.25]),
    ([6.4, -0.6], [0.3, 0.3]),
];

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
/// README showcase: 48 frames, one every 2.4 s, 96 colours (1.69 MB).
const SHOWCASE_LIDAR_FRAMES_PER_GIF_FRAME: u64 = 24;
const SHOWCASE_FRAMES: usize = 48;
const SHOWCASE_POSTER_FRAME: usize = 13;
const SHOWCASE_COLORS: u32 = 96;
const COLORMAP_BUCKETS: usize = 16;

fn repo_path(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

/// Yaw of a world rotation about +y, zero along +x, positive turning left.
fn yaw_of(rotation: Quat) -> f64 {
    let facing = rotation * Vec3::X;
    (-facing.z).atan2(facing.x)
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// Joint angles that put the pad centre at `(x, y, z)` in the base frame: the yaw
/// joint turns the arm toward it, the shoulder and elbow reach it elbow up, and the
/// wrist keeps the pad face level.
fn arm_pose(pad_x_m: f64, pad_y_m: f64, pad_z_m: f64) -> [f64; 4] {
    let yaw = pad_y_m.atan2(pad_x_m - SHOULDER_X_M);
    let radial = (pad_x_m - SHOULDER_X_M).hypot(pad_y_m);
    let u = radial - PAD_FROM_WRIST_M;
    // Pitch angles are positive downward (about +y, x turns toward -z).
    let v = SHOULDER_Z_M - pad_z_m;
    let cos_elbow = ((u * u + v * v - UPPER_M * UPPER_M - FORE_M * FORE_M)
        / (2.0 * UPPER_M * FORE_M))
        .clamp(-1.0, 1.0);
    let elbow = cos_elbow.acos();
    let shoulder = v.atan2(u) - (FORE_M * elbow.sin()).atan2(UPPER_M + FORE_M * elbow.cos());
    [yaw, shoulder, elbow, -(shoulder + elbow)]
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Walk to the doorway's centre line, then face the door.
    Approach(usize),
    ReachOpen,
    PushOpen,
    /// Walk around the open leaf to the start of the closing line.
    GoAround(usize),
    ReachClose,
    PushClose,
    /// Standing beside the nearly shut leaf, sweep the pad out to the right to
    /// press its last degrees shut.
    Nudge,
    StepBack,
    Done,
}

struct Run {
    sim: UrdfSceneSim,
    trot: UnitreeGo2ModelTrot,
    spec: LidarSpec,
    rig: LidarRigOcclusion,
    pattern: LivoxMid360Pattern,
    frame_index: u64,
    frame_start_pose: WorldTransform3,
    slam: Slam2d,
    odom: Pose2d,
    odom_at_frame_start: Pose2d,
    slam_pose: Pose2d,
    odom_at_slam: Pose2d,
    phase: Phase,
    phase_start_s: f64,
    phase_log: Vec<(f64, Phase)>,
    open_pose: [f64; 4],
    close_pose: [f64; 4],
    last_cloud: Option<PointCloud>,
    true_trail: Vec<[f64; 2]>,
    door_max_rad: f64,
    door_contact_steps: BTreeMap<&'static str, u64>,
    slam_error_sq: f64,
    slam_error_max_m: f64,
    error_samples: u64,
    min_clearance_m: f64,
    min_height_m: f64,
}

impl Run {
    fn new() -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&repo_path(
            "assets/scenes/unitree_go2_door.rne.scene.toml",
        ))
        .expect("load door scene");
        let trot = UnitreeGo2ModelTrot::stand_up(&mut sim).with_total_mass_kg(TOTAL_MASS_KG);
        // URDF robots share collision group 1 with self-collision filtered out,
        // which would also keep the Go2 and the door from touching; the door gets
        // its own group.
        assert!(sim.set_named_collision_groups(
            "door_leaf",
            CollisionGroups {
                memberships: 2,
                filter: u32::MAX,
            },
        ));
        // Stand-up sets every motor to hold position; the hinge must swing freely.
        assert!(sim.configure_named_position_motor("door_leaf", 0.0, 0.0, 1.0e-9));
        for (joint, effort) in ARM_JOINTS.iter().zip(ARM_EFFORT_NM) {
            assert!(sim.configure_named_position_motor(joint, 60.0, 4.0, effort));
        }
        let rig: LidarRigOcclusion = serde_json::from_str(
            &fs::read_to_string(repo_path(
                "assets/sensors/livox_mid360/go2_rig_occlusion.json",
            ))
            .expect("read rig table"),
        )
        .expect("parse rig table");
        let frame_start_pose = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let grid = OccupancyGrid::new(
            (MAP_SIZE_M[0] / MAP_RESOLUTION_M) as usize,
            (MAP_SIZE_M[1] / MAP_RESOLUTION_M) as usize,
            MAP_RESOLUTION_M,
            Pose2d::new(MAP_ORIGIN[0], MAP_ORIGIN[1], 0.0),
        )
        .expect("map grid");
        let start = true_pose(&sim);
        let mut run = Self {
            sim,
            trot,
            spec: livox_mid360_spec(),
            rig,
            pattern: LivoxMid360Pattern::new(),
            frame_index: 0,
            frame_start_pose,
            slam: Slam2d::new(grid, SlamConfig::default()),
            odom: start,
            odom_at_frame_start: start,
            slam_pose: start,
            odom_at_slam: start,
            phase: Phase::Approach(0),
            phase_start_s: 0.0,
            phase_log: Vec::new(),
            // Opening: 0.35 m left of the body, toward the hinge; closing: 0.4 m
            // right. Both 0.55 m ahead at the push plate's height.
            open_pose: arm_pose(0.55, 0.35, 0.17),
            close_pose: arm_pose(0.55, -0.40, 0.17),
            last_cloud: None,
            true_trail: Vec::new(),
            door_max_rad: 0.0,
            door_contact_steps: BTreeMap::new(),
            slam_error_sq: 0.0,
            slam_error_max_m: 0.0,
            error_samples: 0,
            min_clearance_m: f64::MAX,
            min_height_m: f64::MAX,
        };
        run.set_arm(STOW_POSE);
        run
    }

    fn set_arm(&mut self, pose: [f64; 4]) {
        let targets: Vec<UrdfJointPositionTarget<'_>> = ARM_JOINTS
            .iter()
            .zip(pose)
            .map(|(link_name, position)| UrdfJointPositionTarget {
                link_name,
                position,
            })
            .collect();
        self.sim.set_joint_position_targets(&targets);
    }

    fn door_rad(&self) -> f64 {
        self.sim
            .named_joint_position("door_leaf")
            .unwrap_or(f64::NAN)
    }

    fn done(&self) -> bool {
        self.phase == Phase::Done || self.trot.time_s() >= MAX_DURATION_S
    }

    fn estimate(&self) -> Pose2d {
        self.slam_pose
            .compose(self.odom_at_slam.inverse().compose(self.odom))
    }

    fn step_frame(&mut self) {
        for _ in 0..STEPS_PER_LIDAR_FRAME {
            self.step_control();
            if self.done() {
                break;
            }
        }
        let end_pose = self
            .sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let sweep = LidarSweep::new(self.frame_start_pose, end_pose);
        let cloud = self.sim.sample_livox_mid360(
            &sweep,
            &self.spec,
            &self.pattern,
            self.frame_index,
            Some(&self.rig),
            SensorNoiseKey::new(self.sim.world_seed(), self.spec.seed, 1, self.frame_index),
        );
        let scan = self.build_scan(&cloud, &sweep);
        let since_slam = self.odom_at_slam.inverse().compose(self.odom);
        if self.slam.scans_processed() == 0
            || since_slam.x_m.hypot(since_slam.y_m) >= KEYFRAME_TRANSLATION_M
            || since_slam.yaw_rad.abs() >= KEYFRAME_ROTATION_RAD
        {
            self.process_keyframe(&scan);
        }
        self.odom_at_frame_start = self.odom;
        self.frame_start_pose = end_pose;
        self.last_cloud = Some(cloud);
        self.frame_index += 1;

        let truth = true_pose(&self.sim);
        let estimate = self.estimate();
        let error = (truth.x_m - estimate.x_m).hypot(truth.y_m - estimate.y_m);
        self.slam_error_sq += error * error;
        self.slam_error_max_m = self.slam_error_max_m.max(error);
        self.error_samples += 1;
    }

    fn process_keyframe(&mut self, scan: &LaserScan2d) {
        let sensor_from_base = Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0);
        let update = self
            .slam
            .process(scan, self.odom, sensor_from_base)
            .expect("slam update");
        self.slam_pose = update.pose;
        self.odom_at_slam = self.odom;
    }

    /// Turns a frame's returns into a 2D scan at the sensor's pose at the frame end.
    fn build_scan(&self, cloud: &PointCloud, sweep: &LidarSweep) -> LaserScan2d {
        let frame_motion = self.odom_at_frame_start.inverse().compose(self.odom);
        let sensor_from_base = Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0);
        let mut ranges = vec![f64::NAN; SCAN_BEAMS];
        let increment = 2.0 * PI / SCAN_BEAMS as f64;
        for (point, time_s) in cloud.points_m.iter().zip(cloud.timestamps_s.iter()) {
            let fraction = (time_s / self.spec.rotation_period_s).clamp(0.0, 1.0);
            // What the driver reports: the return in the sensor frame at emission.
            let sensor = sweep.pose_at(fraction);
            let raw = sensor.rotation.inverse() * (*point - sensor.translation);
            // IMU attitude: remove roll and pitch, keep the sensor's own heading.
            let yaw = yaw_of(sensor.rotation);
            let tilt = Quat::from_rotation_y(yaw).inverse() * sensor.rotation;
            let level = tilt * raw;
            let height = level.y;
            if !(SCAN_MIN_HEIGHT_M..=SCAN_MAX_HEIGHT_M).contains(&height) {
                continue;
            }
            // Leveled axes: +x forward, +y left.
            let at_emission = Vec3::new(level.x, -level.z, 0.0);
            if at_emission.length() < SCAN_MIN_RANGE_M {
                continue;
            }
            // De-skew with odometry: the base pose at emission, relative to the
            // frame end, interpolated along the frame's odometry motion.
            let partial = Pose2d::new(
                frame_motion.x_m * fraction,
                frame_motion.y_m * fraction,
                frame_motion.yaw_rad * fraction,
            );
            let emission_from_end = frame_motion.inverse().compose(partial);
            let base_point = sensor_from_base.transform_point(at_emission);
            let at_end = sensor_from_base
                .inverse()
                .transform_point(emission_from_end.transform_point(base_point));
            let range = at_end.x.hypot(at_end.y);
            let beam = ((at_end.y.atan2(at_end.x) + PI) / increment).floor() as usize % SCAN_BEAMS;
            if ranges[beam].is_nan() || range < ranges[beam] {
                ranges[beam] = range;
            }
        }
        LaserScan2d {
            time_s: self.trot.time_s(),
            frame: FrameId::new("mid360_level"),
            angle_min_rad: -PI + 0.5 * increment,
            angle_increment_rad: increment,
            range_min_m: SCAN_MIN_RANGE_M,
            range_max_m: SCAN_MAX_RANGE_M,
            ranges_m: ranges,
        }
    }

    fn step_control(&mut self) {
        let command = self.door_controller();
        self.trot.step(&mut self.sim, command);
        self.integrate_odometry();
        let observed = self.sim.observe();
        if self.trot.steps().is_multiple_of(125) {
            self.true_trail.push([observed.base_x_m, observed.base_z_m]);
        }
        self.min_height_m = self.min_height_m.min(observed.base_y_m);
        self.min_clearance_m = self
            .min_clearance_m
            .min(clearance_m([observed.base_x_m, observed.base_z_m]));
        self.door_max_rad = self.door_max_rad.max(self.door_rad());
        for link in ROBOT_LINKS {
            if self.sim.named_entities_in_contact("door_leaf", link) {
                *self.door_contact_steps.entry(link).or_default() += 1;
            }
        }
    }

    /// The door sequence, driven by the estimated pose alone.
    fn door_controller(&mut self) -> UnitreeGo2TrotCommand {
        let t = self.trot.time_s();
        let pose = self.estimate();
        let goto = |target: [f64; 2]| -> UnitreeGo2TrotCommand {
            let desired = (target[1] - pose.y_m).atan2(target[0] - pose.x_m);
            let error = wrap_angle(desired - pose.yaw_rad);
            UnitreeGo2TrotCommand {
                forward_speed_m_s: if error.abs() > 0.6 { 0.06 } else { 0.2 },
                yaw_rate_rad_s: (1.5 * error).clamp(-0.5, 0.5),
            }
        };
        let near = |target: [f64; 2], radius_m: f64| {
            (target[0] - pose.x_m).hypot(target[1] - pose.y_m) < radius_m
        };
        let turn_to = |heading: f64| UnitreeGo2TrotCommand {
            forward_speed_m_s: 0.0,
            yaw_rate_rad_s: (1.5 * wrap_angle(heading - pose.yaw_rad)).clamp(-0.5, 0.5),
        };
        let aligned = |heading: f64| wrap_angle(heading - pose.yaw_rad).abs() < 0.03;
        let elapsed = t - self.phase_start_s;
        let (command, next) = match self.phase {
            Phase::Approach(index) => {
                let waypoints = [[1.0, DOORWAY_CENTRE_Y], [1.35, DOORWAY_CENTRE_Y]];
                match waypoints.get(index) {
                    Some(target) => (
                        goto(*target),
                        near(*target, 0.1).then_some(Phase::Approach(index + 1)),
                    ),
                    None => (turn_to(0.0), aligned(0.0).then_some(Phase::ReachOpen)),
                }
            }
            Phase::ReachOpen => (turn_to(0.0), (elapsed > 1.5).then_some(Phase::PushOpen)),
            Phase::PushOpen => {
                // Hold the doorway's centre line while walking through it.
                let heading = (DOORWAY_CENTRE_Y - pose.y_m).clamp(-0.3, 0.3);
                (
                    UnitreeGo2TrotCommand {
                        forward_speed_m_s: PUSH_SPEED_M_S,
                        yaw_rate_rad_s: (1.5 * wrap_angle(heading - pose.yaw_rad)).clamp(-0.5, 0.5),
                    },
                    (pose.x_m > HINGE_NAV[0] + 1.0).then_some(Phase::GoAround(0)),
                )
            }
            Phase::GoAround(index) => {
                let waypoints = [
                    [HINGE_NAV[0] + 1.65, DOORWAY_CENTRE_Y],
                    [HINGE_NAV[0] + 1.65, 0.5],
                    [CLOSING_LINE_X, 0.9],
                ];
                match waypoints.get(index) {
                    Some(target) => (
                        goto(*target),
                        near(*target, 0.12).then_some(Phase::GoAround(index + 1)),
                    ),
                    None => (
                        turn_to(-FRAC_PI_2),
                        aligned(-FRAC_PI_2).then_some(Phase::ReachClose),
                    ),
                }
            }
            Phase::ReachClose => (
                turn_to(-FRAC_PI_2),
                (elapsed > 1.5).then_some(Phase::PushClose),
            ),
            Phase::PushClose => {
                // Hold the closing line heading south.
                let heading = -FRAC_PI_2 - (pose.x_m - CLOSING_LINE_X).clamp(-0.3, 0.3);
                (
                    UnitreeGo2TrotCommand {
                        forward_speed_m_s: CLOSE_SPEED_M_S,
                        yaw_rate_rad_s: (1.5 * wrap_angle(heading - pose.yaw_rad)).clamp(-0.5, 0.5),
                    },
                    (pose.y_m < -1.05).then_some(Phase::Nudge),
                )
            }
            Phase::Nudge => {
                // Beside the body at the leaf's height, then outward to the right.
                let reach = ((elapsed - NUDGE_SETTLE_S) / NUDGE_SWEEP_S).clamp(0.0, 1.0);
                let lateral =
                    NUDGE_START_LATERAL_M + (NUDGE_END_LATERAL_M - NUDGE_START_LATERAL_M) * reach;
                self.set_arm(arm_pose(0.0, lateral, 0.17));
                (
                    UnitreeGo2TrotCommand::default(),
                    (elapsed > NUDGE_SETTLE_S + NUDGE_SWEEP_S + 1.0).then_some(Phase::StepBack),
                )
            }
            Phase::StepBack => (
                UnitreeGo2TrotCommand::default(),
                (elapsed > 4.0).then_some(Phase::Done),
            ),
            Phase::Done => (UnitreeGo2TrotCommand::default(), None),
        };
        if let Some(next) = next {
            match next {
                Phase::ReachOpen => self.set_arm(self.open_pose),
                Phase::GoAround(0) | Phase::StepBack => self.set_arm(STOW_POSE),
                Phase::ReachClose => self.set_arm(self.close_pose),
                _ => {}
            }
            self.phase = next;
            self.phase_start_s = t;
            self.phase_log.push((t, next));
        }
        command
    }

    /// Leg odometry: the body's planar velocity and yaw rate with a scale error
    /// and a yaw-rate bias, integrated at the control rate.
    fn integrate_odometry(&mut self) {
        let observed = self.sim.observe();
        let base = self.sim.named_transform("base").expect("base pose");
        let yaw = yaw_of(base.rotation);
        let (vx, vy) = (
            observed.base_linear_velocity_x_m_s,
            -observed.base_linear_velocity_z_m_s,
        );
        let (sin, cos) = yaw.sin_cos();
        let forward = cos * vx + sin * vy;
        let left = -sin * vx + cos * vy;
        let dt = 1.0 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ;
        let scale = 1.0 + ODOM_SCALE_ERROR;
        let delta = Pose2d::new(
            forward * scale * dt,
            left * scale * dt,
            (observed.base_angular_velocity_y_rad_s + ODOM_YAW_RATE_BIAS_RAD_S) * dt,
        );
        self.odom = self.odom.compose(delta);
    }

    fn slam_rms_m(&self) -> f64 {
        (self.slam_error_sq / self.error_samples.max(1) as f64).sqrt()
    }
}

/// The base's true planar pose in the navigation frame, for scoring only.
fn true_pose(sim: &UrdfSceneSim) -> Pose2d {
    let base = sim.named_transform("base").expect("base pose");
    Pose2d::new(
        base.translation.x,
        -base.translation.z,
        yaw_of(base.rotation),
    )
}

fn clearance_m(position: [f64; 2]) -> f64 {
    OBSTACLES
        .iter()
        .map(|(center, half)| {
            let dx = ((position[0] - center[0]).abs() - half[0]).max(0.0);
            let dz = ((position[1] - center[1]).abs() - half[1]).max(0.0);
            dx.hypot(dz)
        })
        .fold(f64::MAX, f64::min)
}

fn run(mut on_frame: impl FnMut(&Run)) -> Run {
    let mut run = Run::new();
    while !run.done() {
        run.step_frame();
        on_frame(&run);
    }
    run
}

fn report_and_gate(run: &Run) {
    let door_final = run.door_rad();
    let pad_steps = run.door_contact_steps.get(PAD_LINK).copied().unwrap_or(0);
    let other: Vec<_> = run
        .door_contact_steps
        .iter()
        .filter(|(link, _)| **link != PAD_LINK)
        .collect();
    println!(
        "phases: {}",
        run.phase_log
            .iter()
            .map(|(t, phase)| format!("{t:.1}s {phase:?}"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!(
        "door: opened to {:.3} rad ({:.0}°), left at {:.3} rad ({:.1}°); pad touched it for {:.1} s, other links {other:?}",
        run.door_max_rad,
        run.door_max_rad.to_degrees(),
        door_final,
        door_final.to_degrees(),
        pad_steps as f64 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ
    );
    println!(
        "robot: done in {:.1} s, clearance >= {:.2} m, lowest body {:.3} m; localization {:.3} m RMS ({:.3} max)",
        run.trot.time_s(),
        run.min_clearance_m,
        run.min_height_m,
        run.slam_rms_m(),
        run.slam_error_max_m
    );
    assert_eq!(run.phase, Phase::Done, "the sequence did not finish");
    assert!(
        run.door_max_rad > 1.4,
        "door opened only {:.3} rad",
        run.door_max_rad
    );
    // Measured 0.000 rad: the nudge presses the leaf against its stop.
    assert!(door_final < 0.02, "door left open at {door_final:.3} rad");
    assert!(pad_steps > 0, "the pad never touched the door");
    assert!(other.is_empty(), "other links touched the door: {other:?}");
    assert!(
        run.min_height_m > 0.2,
        "the walk sagged: {:.3} m",
        run.min_height_m
    );
    assert!(
        run.min_clearance_m > 0.3,
        "came within {:.2} m of a wall",
        run.min_clearance_m
    );
    assert!(
        run.slam_rms_m() < 0.15,
        "localization RMS {:.3} m",
        run.slam_rms_m()
    );
}

fn main() {
    if std::env::args().any(|arg| arg == "--smoke") {
        let run = run(|_| {});
        report_and_gate(&run);
        println!(
            "smoke ok: the Go2 pushed the door open with its arm, went through, and shut it; digest={:#018x}",
            state_digest(&run)
        );
        return;
    }
    capture_showcase();
}

/// FNV-1a over the bits of the state the run leaves behind: the robot's pose and
/// joints, the door, and the robot's own estimate.
fn state_digest(run: &Run) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    let mut mix = |value: f64| {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    let base = run.sim.named_transform("base").expect("base pose");
    for value in [
        base.translation.x,
        base.translation.y,
        base.translation.z,
        base.rotation.x,
        base.rotation.y,
        base.rotation.z,
        base.rotation.w,
    ] {
        mix(value);
    }
    for joint in ARM_JOINTS.iter().chain(["door_leaf"].iter()) {
        mix(run.sim.named_joint_position(joint).unwrap_or(f64::NAN));
    }
    let estimate = run.estimate();
    for value in [
        estimate.x_m,
        estimate.y_m,
        estimate.yaw_rad,
        run.trot.time_s(),
    ] {
        mix(value);
    }
    hash
}

fn hash_rgba(rgba: &[u8]) -> u64 {
    rgba.iter().fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn sha256_file(path: &Path) -> String {
    Sha256::digest(fs::read(path).expect("read hash input"))
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// Renders the README showcase: a 960 x 540 frame every 2.4 s of the run, the
/// GIF, a poster, and metadata with replay evidence.
fn capture_showcase() {
    let view = CameraOrbit {
        focus: Vec3::new(2.7, 0.2, 0.3),
        yaw_rad: -2.45,
        pitch_rad: 0.72,
        distance_m: 5.2,
    };
    // A headless run first: the capture must replay it exactly.
    let headless = run(|_| {});
    report_and_gate(&headless);
    let headless_digest = state_digest(&headless);
    let initial_digest = state_digest(&Run::new());

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    let frames_dir = repo_path("target/rne-showcase-go2-door");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");
    let mut hashes = Vec::new();
    let mut sampled_sim_steps = Vec::new();
    let mut sampled_phases = Vec::new();
    let captured = run(|run| {
        if !run
            .frame_index
            .is_multiple_of(SHOWCASE_LIDAR_FRAMES_PER_GIF_FRAME)
            || hashes.len() >= SHOWCASE_FRAMES
        {
            return;
        }
        let rgba = render_frame(&mut backend, &camera, &mut mesh_cache, run, &view);
        write_png(
            &frames_dir.join(format!("frame-{:03}.png", hashes.len())),
            &rgba,
        )
        .expect("write frame");
        hashes.push(hash_rgba(&rgba));
        sampled_sim_steps.push(run.trot.steps());
        sampled_phases.push(format!("{:?}", run.phase));
    });
    report_and_gate(&captured);
    let final_digest = state_digest(&captured);
    assert_eq!(
        final_digest, headless_digest,
        "the capture did not replay the headless run"
    );

    let frame_count = hashes.len();
    let unique_render_hashes = hashes
        .iter()
        .collect::<std::collections::BTreeSet<_>>()
        .len();
    let duplicate_adjacent_frames = hashes.windows(2).filter(|pair| pair[0] == pair[1]).count();
    let media_dir = repo_path("docs/media");
    let gif_path = media_dir.join("showcase-go2-door.gif");
    let poster_path = media_dir.join("showcase-go2-door.png");
    let ffmpeg_filter = format!(
        "split[a][b];[a]palettegen=max_colors={SHOWCASE_COLORS}:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle"
    );
    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error", "-framerate", "6", "-i"])
        .arg(frames_dir.join("frame-%03d.png"))
        .args(["-vf", &ffmpeg_filter])
        .arg(&gif_path)
        .status()
        .expect("run ffmpeg");
    assert!(status.success(), "ffmpeg gif encode failed");
    fs::copy(
        frames_dir.join(format!("frame-{SHOWCASE_POSTER_FRAME:03}.png")),
        &poster_path,
    )
    .expect("write poster");

    let door_final = captured.door_rad();
    let metadata = serde_json::json!({
        "kind": "rne_showcase_environment_metadata",
        "schema_version": 1,
        "environment_id": "go2-door",
        "subject": "Go2 with an arm pushing a swing door open, walking through, and shutting it, steered by Mid-360 localization",
        "visual_state_sync": "The Go2, its arm, and the door leaf are drawn from the solved simulation poses each frame. The door is a separate articulated body on a damped hinge moved only by contact; the Mid-360 returns drawn are that frame's scan, and the yellow trail is the robot's true path.",
        "simulation": {
            "scenario": "Go2 door push open, pass, and push shut (examples/132_go2_door)",
            "steps": captured.trot.steps(),
            "initial_state_digest": initial_digest,
            "final_state_digest": final_digest,
            "replay_final_state_digest": headless_digest,
            "replay_match": final_digest == headless_digest,
            "outcome": format!(
                "door_max_rad={:.3}; door_final_rad={:.3}; pad_contact_s={:.1}; other_link_contacts={}; localization_rms_m={:.3}; min_clearance_m={:.2}; lowest_body_m={:.3}",
                captured.door_max_rad,
                door_final,
                captured.door_contact_steps.get(PAD_LINK).copied().unwrap_or(0) as f64
                    / UNITREE_GO2_MODEL_TROT_CONTROL_HZ,
                captured.door_contact_steps.iter().filter(|(link, _)| **link != PAD_LINK).count(),
                captured.slam_rms_m(),
                captured.min_clearance_m,
                captured.min_height_m
            ),
        },
        "capture": {
            "gpu_rendered": true,
            "width_px": WIDTH,
            "height_px": HEIGHT,
            "frame_count": frame_count,
            "frame_pattern": "target/rne-showcase-go2-door/frame-%03d.png",
            "gif_path": "docs/media/showcase-go2-door.gif",
            "gif_bytes": fs::metadata(&gif_path).expect("gif").len(),
            "gif_sha256": sha256_file(&gif_path),
            "poster_path": "docs/media/showcase-go2-door.png",
            "poster_bytes": fs::metadata(&poster_path).expect("poster").len(),
            "poster_sha256": sha256_file(&poster_path),
            "poster_frame": SHOWCASE_POSTER_FRAME,
            "sampled_sim_steps": sampled_sim_steps,
            "sampled_phases": sampled_phases,
            "unique_render_hashes": unique_render_hashes,
            "duplicate_adjacent_frames": duplicate_adjacent_frames,
            "ffmpeg_command": format!("ffmpeg -y -framerate 6 -i target/rne-showcase-go2-door/frame-%03d.png -vf \"{ffmpeg_filter}\" docs/media/showcase-go2-door.gif"),
        },
        "camera": {
            "fov_y_rad": std::f64::consts::FRAC_PI_4,
            "yaw_rad": view.yaw_rad,
            "pitch_rad": view.pitch_rad,
            "distance_m": view.distance_m,
        },
        "provenance": [
            "assets/scenes/unitree_go2_door.rne.scene.toml",
            "assets/robots/unitree_go2_arm.rne.robot.toml",
            "assets/robots/swing_door.rne.robot.toml",
            "assets/sensors/livox_mid360/go2_rig_occlusion.json",
            "examples/132_go2_door/main.rs",
        ],
        "reproduce_smoke": "cargo run --locked -p go2_door --example 132_go2_door -- --smoke",
        "reproduce_capture": "cargo run --release --locked -p go2_door --example 132_go2_door -- --capture",
    });
    fs::write(
        media_dir.join("showcase-go2-door.json"),
        serde_json::to_string_pretty(&metadata).expect("metadata json") + "\n",
    )
    .expect("write metadata");
    println!(
        "wrote {} ({frame_count} frames, {} bytes)",
        gif_path.display(),
        fs::metadata(&gif_path).expect("gif").len()
    );
}

fn render_frame(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    run: &Run,
    view: &CameraOrbit,
) -> Vec<u8> {
    let mut scene = build_visual_render_scene(run.sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { size_m } if size_m.x >= 20.0));
    scene.items.push(box_item(
        Vec3::new(2.3, -0.005, 0.0),
        Vec3::new(9.6, 0.01, 5.0),
        [0.08, 0.09, 0.11, 1.0],
    ));
    let mut trail = QuadMesh::default();
    for pair in run.true_trail.windows(2) {
        trail.add_segment(
            Vec3::new(pair[0][0], 0.012, pair[0][1]),
            Vec3::new(pair[1][0], 0.012, pair[1][1]),
            0.025,
        );
    }
    push_mesh(&mut scene, trail, [0.98, 0.84, 0.25, 1.0]);
    if let Some(cloud) = &run.last_cloud {
        append_height_colored_points(&mut scene, cloud);
    }
    // The Mid-360 itself, hanging upside down from its mount.
    let mount = run
        .sim
        .named_mount_transform("base", &unitree_go2_mid360_mount())
        .expect("mount pose");
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation: mount.translation,
            rotation: mount.rotation * Quat::from_rotation_x(FRAC_PI_2),
            scale: Vec3::new(0.065, 0.065, 0.06),
        },
        shape: VisualShape::Cylinder {
            radius_m: 0.5,
            length_m: 1.0,
        },
        color_rgba: [0.12, 0.12, 0.13, 1.0],
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    });
    let roots: Vec<&Path> = run
        .sim
        .mesh_package_roots()
        .iter()
        .map(PathBuf::as_path)
        .collect();
    mesh_cache
        .resolve_scene(&mut scene, &roots)
        .expect("resolve official Go2 meshes");
    backend
        .render_scene_camera(camera, &view.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render frame")
        .color
        .rgba8
}

fn append_height_colored_points(scene: &mut RenderScene, cloud: &PointCloud) {
    let mut buckets: Vec<QuadMesh> = (0..COLORMAP_BUCKETS).map(|_| QuadMesh::default()).collect();
    for point in &cloud.points_m {
        let t = point.y.clamp(0.0, 1.0);
        let bucket =
            ((t * (COLORMAP_BUCKETS - 1) as f64).round() as usize).min(COLORMAP_BUCKETS - 1);
        buckets[bucket].add_marker(*point, 0.02);
    }
    for (bucket, mesh) in buckets.into_iter().enumerate() {
        let t = bucket as f64 / (COLORMAP_BUCKETS - 1) as f64;
        push_mesh(scene, mesh, turbo_colormap(0.1 + 0.85 * t));
    }
}

fn box_item(translation: Vec3, scale: Vec3, color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    fn add_marker(&mut self, center: Vec3, radius_m: f64) {
        let x = Vec3::X * radius_m;
        let y = Vec3::Y * radius_m;
        let z = Vec3::Z * radius_m;
        self.add_quad(center - x, center + y, center - y, center + x);
        self.add_quad(center - z, center + y, center - y, center + z);
        self.add_quad(center - x, center + z, center - z, center + x);
    }

    fn add_segment(&mut self, start: Vec3, end: Vec3, width_m: f64) {
        let delta = end - start;
        let side = Vec3::new(-delta.z, 0.0, delta.x).normalize_or_zero() * width_m * 0.5;
        self.add_quad(start - side, start + side, end - side, end + side);
    }

    fn add_quad(&mut self, first: Vec3, second: Vec3, third: Vec3, fourth: Vec3) {
        let base = self.positions.len() as u32;
        self.positions
            .extend([first, second, third, fourth].map(|p| [p.x as f32, p.y as f32, p.z as f32]));
        self.normals.extend([[0.0, 1.0, 0.0]; 4]);
        self.texcoords.extend([[0.0, 0.0]; 4]);
        // Both windings, so the quad shows from either side.
        self.indices.extend([
            base,
            base + 1,
            base + 2,
            base + 2,
            base + 1,
            base + 3,
            base,
            base + 2,
            base + 1,
            base + 2,
            base + 3,
            base + 1,
        ]);
    }
}

fn push_mesh(scene: &mut RenderScene, mesh: QuadMesh, color_rgba: [f32; 4]) {
    if mesh.indices.is_empty() {
        return;
    }
    scene.items.push(RenderSceneItem {
        transform: Transform3::IDENTITY,
        shape: VisualShape::DynamicMesh,
        color_rgba,
        mesh: Some(Arc::new(TriangleMesh {
            positions: mesh.positions,
            normals: mesh.normals,
            texcoords: mesh.texcoords,
            indices: mesh.indices,
            skinning: None,
        })),
        base_color_texture: None,
        material: Default::default(),
    });
}

/// Google's Turbo colormap, polynomial approximation.
fn turbo_colormap(t: f64) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let polynomial = |c: [f64; 6]| -> f32 {
        (c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))).clamp(0.0, 1.0) as f32
    };
    [
        polynomial([
            0.135_721_38,
            4.615_392_60,
            -42.660_322_58,
            132.131_082_34,
            -152.942_393_96,
            59.286_379_43,
        ]),
        polynomial([
            0.091_402_61,
            2.194_188_39,
            4.842_966_58,
            -14.185_033_33,
            4.277_298_57,
            2.829_566_04,
        ]),
        polynomial([
            0.106_673_30,
            12.641_946_08,
            -60.582_048_36,
            110.362_767_71,
            -89.903_109_12,
            27.348_249_73,
        ]),
        1.0,
    ]
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}
