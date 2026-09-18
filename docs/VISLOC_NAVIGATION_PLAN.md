# visloc-rs navigation integration plan

Status: design confirmed (2026-09-18). Target: run **RoboSim/RNE + visloc-rs** up to
navigation, camera-based, with a headless deterministic test and a rendered demo.

## Confirmed decisions

1. **Robot / scene**: differential-drive mobile base (e.g. office AGV), indoor
   textured scene (`assets/scenes/office_agv_delivery.rne.scene.toml`, or a 3DGS
   interior). Camera localization needs texture.
2. **Localization mode**: **relocalize against a prebuilt map**
   (`visloc_rs::localization`) for the closed-loop run; the map is built offline
   from a RoboSim episode (no ground truth used to build it).
3. **Sensors**: **stereo + IMU** (Basalt VI-SLAM / VIO). RNE's diff-drive robot
   has no camera asset, so a stereo pair is synthesized by sampling
   `sample_camera_rgbd_keyed` at two poses offset by ±baseline/2 along the camera
   x-axis (same rotation).
4. **Navigation**: **RNE-native** (`rne_nav` grid planner + DWA / pure-pursuit,
   `rne_nav::drive` differential drive). ROS 2 / Nav2 is a later adapter option.
5. **Deliverables**: (a) headless deterministic test + evidence (localization
   error, goal reached, zero collisions, replay), and (b) a rendered demo.

## Boundary

Add the integration as a **separate example crate / adapter**, never in
`rne_core` (mirrors the ROS 2 rule). It depends on `rne_ai`, `rne_sensor`,
`rne_data`, `rne_nav`, `rne_world`, `rne_math`, and `visloc-rs`. Keep the RNE
workspace `publish = false` for that crate unless it enters
`PUBLIC_RELEASE_PACKAGES`.

## Phases

### M1 — offline capture + visloc localization (no navigation)

Concrete first slice:

1. **Capture example** (new RNE example, modelled on
   `examples/73_diff_drive_dataset_capture`): run a diff-drive episode, sample a
   **stereo pair + IMU**, and export an **EuRoC-format `mav0/`**:
   - `mav0/cam0/{data.csv,data/*.png}`, `mav0/cam1/...` (camera at 20 Hz,
     `CAMERA_PERIOD_STEPS = 3` at the 1/60 s fixed step)
   - `mav0/imu0/data.csv` (IMU at the 60 Hz fixed step)
   - `gt.tum` (RNE ground-truth `base_link` pose per frame)
   - a Basalt Double-Sphere calibration (`xi = 0`, `alpha = 0`, pinhole) built
     from `CameraSpec { width, height, fov_y_rad }` with
     `fx = fy = (height/2)/tan(fov_y/2)`, principal point at the image centre,
     and `T_imu_cam` from the camera mount offset.
   Relevant APIs: `rne_ai::{DiffDriveEpisode, DiffDriveEpisodeConfig,
   DiffDriveAction, DiffDriveSim, build_diff_drive_render_scene}`,
   `rne_sensor::{sample_camera_rgbd_keyed, CameraSpec, CameraDistortion,
   sample_imu, ImuSpec}`, `rne_render::HeadlessRenderBackend`,
   `rne_world::Transform3`.
   (The current `dataset_diff_drive` scene is a plain room; a textured scene is
   required for a usable VIO — this is the main unknown to settle first.)

2. **visloc-rs run** (visloc-rs checkout): feed the exported `mav0/` to
   `basalt_euroc_vio_demo` (VIO) and `basalt_mapper_offline_demo` (map), then
   evaluate the estimated trajectory against `gt.tum` (SE(3)/Sim(3) ATE) with
   the existing scripts. No navigation yet.

Exit criteria: RNE episode → EuRoC export → visloc trajectory with a documented
ATE, deterministic on replay with a fixed `rng_seed`.

### M2 — online relocalization

Run `visloc_rs::localization` per camera frame against the M1 map; produce
`map → base_link`; bound per-frame latency and verify determinism. Output a
`Pose2d` (planar projection) plus full SE(3) for the TF chain.

### M3 — close the navigation loop

Feed the visloc pose into `rne_nav` (`TfBuffer`: map → odom → base_link;
`NavMap` for the costmap), plan a path to a goal waypoint with `rne_nav`'s grid
planner and follow it with DWA / pure-pursuit, applying `VelocityCommand2d`
through `rne_nav::drive` to the differential-drive base. Headless goal-reaching
scenario with metrics: goal reached, path-following error, zero collisions.

### M4 — evidence, docs, demo

Deterministic replay evidence, localization/goal metrics, docs, and a rendered
`wgpu` demo (option 5b). Optional follow-ups: mobile manipulator, fisheye camera,
ROS 2 / Nav2 adapter.

## Determinism / contract

- Fixed `rng_seed`, `SimClock` only (no wall clock), explicit seeds for sensor
  noise; visloc-rs Basalt is deterministic.
- Frame convention: RNE is Y-up; visloc/Basalt is Z-up. Map the camera/IMU world
  frame explicitly and record the convention in the map build (relocalization
  must reuse it).
- Builds go under `E:\RNE-build\visloc-nav` ; large datasets on `E:`.

## Notes

- RNE already has native 2D LiDAR SLAM (`rne_slam`) and Nav2 via the ROS 2
  adapter; this integration is specifically **camera-based** and closed through
  `rne_nav`, not ROS 2.
