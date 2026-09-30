# Robot Native Engine

**Robots are not plugins.** RNE is a Rust robot-native game engine for deterministic
simulation, embodied AI, synthetic sensors, and policy evaluation.

[![Release](https://img.shields.io/github/v/release/rsasaki0109/RobotNativeEngine)](https://github.com/rsasaki0109/RobotNativeEngine/releases)
[![CI](https://github.com/rsasaki0109/RobotNativeEngine/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/RobotNativeEngine/actions/workflows/ci.yml)

RNE combines a headless, replayable simulation core with real wgpu rendering.
Worlds hold robot, sensor, actuator, agent, and episode entities; simulation
needs no renderer, and ROS 2 is an optional adapter, not a core dependency.

## Real simulation showcase

Every frame below is rendered by wgpu from deterministic simulation or pinned
camera state; gates and regeneration commands are in
[README showcase acceptance](docs/README_SHOWCASE.md).

<table>
  <tr>
    <td colspan="2" align="center">
      <picture>
        <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/house-mobile-manipulation.png">
        <img src="docs/media/house-mobile-manipulation.gif" alt="PBR mobile manipulator grasping, lifting, carrying, and placing an object in a real captured indoor 3DGS environment with live wrist RGB-D and a 2D task trace" width="900">
      </picture>
      <br><b>Real indoor 3DGS · mobile manipulation</b><br>
      <sub>A real photo-derived interior (Voxel51 Dr Johnson 3DGS) bound to real cameras and landmarks by a fail-closed validation fixture. A SCARA arm on a lift mast closes its pads on the block, which is held where they caught it (the pads stay level with it, within 8 mm of its faces, measured every step), lifted 0.501 m, carried 1.596 m and placed within 0.061 m; live wrist RGB-D self-masks the robot and drives the final approach without payload truth. <a href="docs/media/house-mobile-manipulation.json">metadata</a> · <a href="examples/89_house_mobile_lift_hero/main.rs">source</a></sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <picture>
        <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/showcase-openarm.png">
        <img src="docs/media/showcase-openarm.gif" alt="Official OpenArm v2 bimanual robot picking a block, handing it to the other gripper, and placing it on a target pad under delayed joint-feedback control with live telemetry" width="460">
      </picture>
      <br><b>OpenArm v2 · bimanual control</b><br>
      <sub>18-axis typed feedback, an IK-solved pick / handoff / place cycle gated on real fingertip contact, and exact Rapier replay over 1,400 steps. <a href="docs/media/showcase-openarm.json">metadata</a> · <a href="examples/90_showcase_captures/openarm.rs">source</a></sub>
    </td>
    <td width="50%" align="center">
      <picture>
        <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/showcase-factory.png">
        <img src="docs/media/showcase-factory.gif" alt="Unitree G1 humanoid at a belt conveyor, lowering its right hand onto each part the belt stops in front of it, with a lamp stack turning green as each part is touched" width="460">
      </picture>
      <br><b>Factory inspection</b><br>
      <sub>Parts ride a belt conveyor, a kinematic belt carrying free dynamic parts by friction, and stop in front of the G1. It lowers its right hand onto each part until the simulation reports contact, rests it there and lifts away; a lamp turns green only on that contact. The fingertips meet each part within about 1 cm of its top-face centre without moving it. <a href="docs/media/showcase-factory.json">metadata</a></sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <picture>
        <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/showcase-go2-door.png">
        <img src="docs/media/showcase-go2-door.gif" alt="A Unitree Go2 with an arm on its back pushes a swing door open with the pad on its arm, walks through the doorway, walks around the open door and pushes it shut from the other side, with its Livox Mid-360 returns drawn coloured by height" width="460">
      </picture>
      <br><b>Go2 · door with an arm</b><br>
      <sub>The Go2 pushes a free-swinging 6 kg door open with the pad on its arm, walks through, and pushes it back against its stop; the pad is the only part of the robot that ever touches the door. It steers only on its own Livox Mid-360 localization (0.05 m RMS), with the sensor model fitted to real Go2 recordings. <a href="docs/media/showcase-go2-door.json">metadata</a> · <a href="docs/GO2_DOOR.md">details</a></sub>
    </td>
    <td width="50%" align="center">
      <picture>
        <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/showcase-uav.png">
        <img src="docs/media/showcase-uav.gif" alt="Controlled quadrotor flying over a PLATEAU city model with onboard RGB and depth camera views" width="460">
      </picture>
      <br><b>PLATEAU UAV · RGB-D flight</b><br>
      <sub>A detailed multirotor flies 56.0 m over imported PLATEAU LOD1 buildings with textured facades, 2.55 m minimum building clearance, zero collisions, and synchronized onboard RGB-D. <a href="docs/media/showcase-uav.json">metadata</a> · <a href="examples/46_plateau_drone_gif/main.rs">source</a></sub>
    </td>
  </tr>
</table>

## Highlights

| Area | Included | Docs |
| --- | --- | --- |
| City simulation | PLATEAU import, traffic, LiDAR, RGB-D, OSM HUD | [docs](docs/PLATEAU_IMPORT.md), ex. 46–47 |
| Vehicle dynamics | Bicycle/Ackermann, tire saturation, suspension, road excitation | [docs](docs/VEHICLE_DYNAMICS.md), ex. 49–51 |
| Quadruped locomotion | Official Go2, model-based trot and heading control, torque control, disturbances | [docs](docs/GO2_LOCOMOTION.md), ex. 52–65, 126 |
| Humanoid locomotion | Official G1 23-DoF, balance, learned stride, CEM eval | [docs](docs/G1_LOCOMOTION.md), ex. 39, 63, 67, 68 |
| Manipulation | PBR/3DGS mobile manipulator, friction grasp, Dex3 hands | [docs](docs/README_SHOWCASE.md), ex. 32, 40–42, 89 |
| Deformables | XPBD cable and cloth, deterministic headless replay | ex. 43–45 |
| More demos | Localization, native planning/dynamics/legged/WBC/OC, Go2 jump | [docs](docs/DEMOS.md) |

## Independent validation wanted

RNE remains below 1.0 until outside projects reproduce tasks and pass the
shipped conformance kits (native bundles include the tools; no source
checkout needed).

Only [v0.4.0 official
assets](https://github.com/rsasaki0109/RobotNativeEngine/releases/tag/v0.4.0) qualify;
if that page lacks the native archives and `SHA256SUMS` yet, prepare the
checklist but do not open an evidence issue (v0.1.0 does not qualify).

- [External project reproduction + Failure Capsule](https://github.com/rsasaki0109/RobotNativeEngine/issues/new?template=external-project-evidence.yml)
- [Installed flagship reproduction](https://github.com/rsasaki0109/RobotNativeEngine/issues/new?template=installed-flagship-reproduction.yml)
- [Third-party plugin conformance](https://github.com/rsasaki0109/RobotNativeEngine/issues/new?template=third-party-plugin-evidence.yml)
- [External physics/simulator/hardware/accelerator conformance](https://github.com/rsasaki0109/RobotNativeEngine/issues/new?template=external-system-evidence.yml)

See the [external evidence intake guide](docs/EXTERNAL_EVIDENCE_INTAKE.md).
Opening an issue is only the start of review: it does not imply acceptance;
in-repo reference implementations do not count as independent evidence.

## Vehicle dynamics at the grip limit

![Two GT coupés, green kinematic and orange tire-limited dynamic, take the same fast left-hand sweeper under the same pure-pursuit controller; the green car holds the line while the orange one runs wide across the blue runoff, its trail turning red where the front axle saturates](docs/media/vehicle-dynamics.gif)

*Same controller, two plants: the dynamic car's trail turns red once the front axle saturates.* No-slip follows the line; the dynamic car runs wide past tire grip. [Vehicle dynamics](docs/VEHICLE_DYNAMICS.md).

![Four open-wheel cars race on a circuit with kerbs, tyre walls and a grandstand, filmed from trackside camera posts; the faster cars pass on the straights](docs/media/car-race.gif)

*Four cars race three laps on the tire-limited dynamic bicycle model, from a
grid in reverse order of pace. Each follows a minimum-curvature racing line at
the speed its own grip and power allow; a faster car catches a slower one,
takes its tow down the straight and passes beside it, and the car behind
always leaves room. Four passes, the fastest car wins, the closest two cars
came was 2.47 m centre to centre, and no car left the track. The cars are
built from their parts (wings, sidepods, halo, steered and spinning wheels);
the circuit uses CC0 Poly Haven asphalt, grass, tyres and barriers.
[source](examples/128_car_race/main.rs)*
![Four racing quads fly a night course of LED gates, trailing light in their team colours; the faster drones, started last, pass the slower ones on the final lap](docs/media/uav-race.gif)

*Four racing drones fly three laps of an eight-gate course on the
`MultirotorFlight` model (position loop, velocity loop, tilt-limited
acceleration), each chasing a point on the course at the speed its own
curvature profile allows, in its own lane of the gate opening. A pursuit
start sends the slowest off first and the fastest last; all six pairs change
places on the final lap and the fastest wins. All 99 gate crossings are
inside the opening, the tightest with 0.56 m between props and frame, and
the closest two drones came was 0.80 m. The quads are built from their parts
(carbon X frame, motor bells, spinning three-blade props, tilted FPV camera,
battery, antennas, LED strips). [source](examples/129_uav_race/main.rs)*

## Navigation, SLAM, and multi-robot

![Office AGV sharing a corridor with a second AGV, a pedestrian and a hand truck: it swings out to pass the AGV, stops for the pedestrian crossing, and routes around the hand truck to the desk, with its LiDAR returns, tracks and costmap drawn on the floor](docs/media/showcase-nav.gif)

*Nothing but the walls and the desk is on the orange AGV's map. The second AGV,
the pedestrian and the hand truck are bodies in the physics world, so its LiDAR
hits them: red dots are the returns the map does not explain, and yellow rings
are the tracks `ObstacleTracker` makes of them. The route (magenta) is
`plan_path` over a costmap with each track written in, moving ones swept 2 s
ahead, and replanned five times a second. Pure pursuit drives the wheels, and
`avoid_velocities` gives way to the moving tracks. The AGV swings out past the
second AGV (footprints at least 0.32 m apart), stops for 2.2 s while the
pedestrian crosses (0.23 m clear), and docks 0.05 m from the goal. The second
AGV keeps its lane and never needs to brake. It has no simulated sensors: it
gets the orange AGV's pose over the fleet link and the pedestrian's true
position.*

![A robot maps the same warehouse on four days while pallets move, its LiDAR rays and returns drawn live around it; the lifelong map on the board above the far wall updates each evening, with vanished pallets in red and new ones in green](docs/media/lifelong-slam.gif)

*Lifelong SLAM: the robot maps this warehouse on four days, starting somewhere
new each time with 0.6 to 1.6 m of odometry drift over its loop, while pallets
arrive, leave and move. Each day it recognizes where it is in the lifelong map,
registers every keyframe against it, and the map on the board updates:
red where a pallet left, green where one arrived. Every pallet that changed was
detected on every day (9 of 9), and 798 of the 828 cells flagged as changed
lie on a pallet that really changed. The map stays within 3 cm of the building
after rigid alignment, its frame holds where the first day put it, and pruning
keeps the pose graph at the first day plus the latest. The cyan rays, the red
outline and the yellow returns are the scan the robot's LiDAR returns at that
moment. The board and floor marks are drawn from the lifelong map itself.
[Lifelong mapping](docs/SLAM.md#lifelong-mapping-across-sessions),
[source](examples/127_lifelong_slam/main.rs).*

`rne_nav`/`rne_slam`: deterministic, ROS-free costmaps, a transform tree,
A*/DWA/pure-pursuit, multi-robot avoidance, an EKF, 3D ICP, and online 2D
SLAM with loop closure and AMCL (a ROS 2 adapter maps to Nav2). Details:
[Navigation](docs/NAVIGATION.md), [SLAM](docs/SLAM.md).

## Logistics across floors

<p align="center">
  <img src="docs/media/warehouse-logistics.gif" alt="Forklift AGV lifting a case off a goods-in stand, carrying it into a lift, riding to the upper floor and setting it down on an outbound stand" width="720">
</p>

**Goods-in to delivery.** A forklift AGV takes a case off a stand, calls the
lift, rides up with the load and sets it down on the floor above. The mast is a
prismatic joint with a position servo and the case is an ordinary dynamic body
throughout: it moves 0.038 m on the tines across the whole carry.
[source](examples/123_warehouse_logistics/main.rs) · more lift demos in
[Multi-floor navigation](docs/MULTI_FLOOR_NAVIGATION.md) and
[More demos](docs/DEMOS.md#two-trucks-one-lift)

## Go2 locomotion

<p align="center">
  <img src="docs/media/go2-heading-steer.gif" alt="The official Go2 trotting head first along an S: straight, a left 90-degree turn, straight, a right 90-degree turn, straight, with its path drawn on the floor coloured by the commanded heading and a yellow arrow showing the heading it is steering to" width="820">
</p>

The official Go2 walks head first through an S on a model-based trot of the
kind Pinocchio-based quadruped stacks run: 500 Hz joint torques, foot
Jacobians from the link frames, stance legs pushing `tau = -J^T f` for weight,
height, speed and yaw rate, swing feet placed by the Raibert heuristic and
tracked by Cartesian PD. A heading loop steers it; held headings stay within
0.04 rad RMS (0.15 rad worst) while it covers 8.5 m. This runs on
`unitree_go2_jump`, the Go2 with its feet attached: the older walking asset
leaves them on the floor ([#346](https://github.com/rsasaki0109/RobotNativeEngine/issues/346)),
and the results measured on it are marked as such in
[docs/GO2_LOCOMOTION.md](docs/GO2_LOCOMOTION.md).
[source](examples/126_go2_heading_steer/main.rs)

<p align="center">
  <picture>
    <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/go2-mid360-room.png">
    <img src="docs/media/go2-mid360-room.gif" alt="The Go2 trots around a table in a room, steering to waypoints, while the upside-down Livox Mid-360 on its back scans: each 0.1 s frame's returns are drawn coloured by height, showing rings on the floor, the table's side, and the walls" width="720">
  </picture>
</p>

The same trot steers to waypoints around a room while an upside-down Livox
Mid-360 on its back scans at 10 Hz. The sensor model is fitted to real Go2
recordings: its non-repetitive four-line pattern matches held-out real returns
to 0.13° median, and the rig occlusion, self returns, and near-range blanking
are measured. While the Go2 walks the loop, the no-return fraction of every
floor-facing elevation band stays between the two recordings'
([docs/LIVOX_MID360.md](docs/LIVOX_MID360.md)).
[source](examples/130_go2_mid360_room/main.rs)

<p align="center">
  <picture>
    <source media="(prefers-reduced-motion: reduce)" srcset="docs/media/go2-mid360-navigation.png">
    <img src="docs/media/go2-mid360-navigation.gif" alt="Cutaway view of two rooms joined by a doorway: the Go2 walks from one room through the doorway to a goal in the far corner of the other and back, while the occupancy map its Mid-360 builds grows on the floor, known free space lighter than unexplored floor and walls marked in pink, with its planned path in yellow" width="720">
  </picture>
</p>

With no map given, the Go2 navigates between two rooms on what its Mid-360
sees: each frame's returns, in the sensor frame at their emission times, are
levelled by IMU attitude, de-skewed by leg odometry, and cut into a 2D scan for
`rne_slam`'s online SLAM; A* on the inflated map plans through unexplored
space and replans every second, so the robot finds the doorway as the partition
appears. It reaches both goals (0.25 m and 0.28 m from them) with 0.042 m RMS
localization error while leg odometry alone drifts 7.1 m.
[source](examples/131_go2_mid360_navigation/main.rs)

With an arm on its back, the Go2 walks through a swing door and shuts it
behind itself (the showcase above). Nothing holds or drives the 6 kg door: the pad on the arm
pushes it open to 87° and back against its stop, and no other part of the
robot ever touches it. Every command comes from the Mid-360 localization
above (0.052 m RMS). [docs/GO2_DOOR.md](docs/GO2_DOOR.md) ·
[source](examples/132_go2_door/main.rs)

## G1 locomotion

![The official Unitree G1 completing a backflip in native RoboSim/Rapier dynamics and landing on its feet](docs/media/unitree-g1-robosim-native-backflip.gif)

*A full backflip in native RoboSim/Rapier: 62.5 µs step, 21 convex body colliders with self-collision, bounded joint effort and gravity only — no imposed base trajectory, no root wrench, no RL. It lands on its feet and is still standing 15 s later. Peak joint speed is 1.039x the URDF rating, under the unchanged 1.05 gate.*

The GIF replays a recorded native rollout — the renderer applies the recorded
poses and takes zero physics ticks, and the model and recording hashes are
checked before the first frame. The controller comes from a parameter search,
not a learned policy. This is a simulator result; hardware is unvalidated.
Details and the full evidence trail:
[docs/G1_CONTACT_BACKFLIP.md](docs/G1_CONTACT_BACKFLIP.md).

Walking is a separate and much weaker claim. Example 68 holds the [v0.3
sustained envelope](docs/media/unitree-g1-sustained-walk.gif) upright for 3000
ticks / 50 s, turning the commanded way the whole time (+1.6 / −2.2 rad)
without holding its heading target, but **that walk goes backwards**: the knees
bend toward the way the robot faces while the body travels the other way,
because the search that found its torque overlay scored distance without a
direction. Measured along the facing, its 8 s windows are -0.16 m and -0.22 m.

[`UnitreeG1TorqueOverlay::FORWARD_STRIDE`](examples/124_g1_forward_stride/main.rs)
walks forwards, straight and without turning: +0.14 to +0.16 m per window,
travel within a mean 0.20 rad of the facing. It holds only under the exact
conditions it was trained in. A constant 1e-6 N·m of extra hip-yaw torque
tips it over, so it cannot yet be steered or stopped. This is a
stability-and-direction claim, not a navigation one. Details:
[docs/G1_LOCOMOTION.md](docs/G1_LOCOMOTION.md).

## Quickstart

```bash
git clone https://github.com/rsasaki0109/RobotNativeEngine.git
cd RobotNativeEngine
cargo run -p hello_world --example 00_hello_world
cargo run -p falling_cube --example 01_falling_cube
```

For a complete local validation, run `cargo run -p xtask -- ci` (the long
smoke gate splits into `manipulator`/`locomotion`/`assets`/`media`
partitions, e.g. `cargo run -p xtask -- ci-smoke media`). The headless asset
CLI, replay, and determinism-check commands, and the full example index, are
in [examples/README.md](examples/README.md).

To poke at a robot by hand, the [Robot workbench](docs/ROBOT_WORKBENCH.md)
puts joint sliders, a floor/obstacle editor, and RGB/depth/LiDAR views for a
URDF or MJCF model in one browser window:
`cargo run --release --locked -p robot_workbench`.

<img src="docs/media/robot-workbench.png" alt="Robot workbench showing SO-101 joint controls, scene editing, RGB, depth and LiDAR views" width="460">

## Independent integrations

The native release archive includes a one-command installed product proof:

```bash
./bin/rne-flagship-proof flagship-proof --cross-backend \
  --measure-on "lab-workstation-a" --verify-installed-bundle .
```

It runs the same indoor TaskSpec through Rapier and bundled MuJoCo, verifies
both replays plus the Failure Capsule against `SHA256SUMS`, and writes a
SHA-256-bound report with no source checkout, renderer, or network needed.
Details: [flagship validation](docs/FLAGSHIP_VALIDATION_WORKFLOW.md).

Third-party plugins, physics backends, adapters, and external task
reproductions go through the fixed
[external evidence intake](docs/EXTERNAL_EVIDENCE_INTAKE.md); submission
never implies acceptance.

## Architecture

The workspace is split by responsibility:

- `rne_core`/`rne_math`/`rne_ecs`/`rne_world`/`rne_robot`/`rne_sensor`/`rne_ai`/`rne_data`: schedules, ECS, spatial math, entity/robot control, sensors, learning interfaces, typed data streams.
- `rne_planning`/`rne_dynamics`/`rne_legged`/`rne_wbc`: backend-neutral joint-space planning, articulated dynamics, legged templates, whole-body control.
- `rne_physics`/`rne_physics_rapier` and `rne_render`/`rne_render_wgpu`: backend-neutral traits plus the Rapier and wgpu implementations.
- `rne_asset`/`rne_plugin`/`rne_traffic`: assets, plugin interfaces, backend-neutral traffic.
- `adapters/ros2`: ROS 2 integration; core crates remain ROS 2-free.

## Determinism and testing

Simulation uses `SimClock`, explicit seeds, stable entity ordering, and
replay digests; headless examples/tests never initialize a renderer; public
APIs use explicit units (`_m`, `_rad`, `_s`, `_hz`); physics backends never
leak engine-specific handles through core traits.

Standard checks:

```bash
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo run -p xtask -- ci-headless
cargo run --locked -p xtask -- flagship
cargo run -p xtask -- ci
```

## Python and ROS 2 adapters

The Python adapter exposes native environments for policy experiments:

```bash
python3 -m venv .venv
.venv/bin/pip install maturin
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml
.venv/bin/python examples/04_python_policy/run.py
```

ROS 2 is optional, isolated under [adapters/ros2](adapters/ros2); see the
[bridge README](adapters/ros2/rne_ros2_bridge/README.md) for setup.

## Documentation

- [Architecture](docs/architecture/000_overview.md) · [Roadmap](docs/ROADMAP.md) · [OSS parity](docs/OSS_PARITY.md) · [Plugin SDK](docs/PLUGIN_SDK.md) · [Browser viewer](web/rne_web_viewer/README.md)
- Conformance/readiness: [physics](docs/EXTERNAL_PHYSICS_BACKEND_CONFORMANCE.md) · [hardware](docs/HARDWARE_ADAPTER_CONFORMANCE.md) · [simulator](docs/EXTERNAL_SIMULATOR_ADAPTER_CONFORMANCE.md) · [OpenArm cross-sim](docs/OPENARM_CROSS_SIM_PROOF.md) · [compat corpus](docs/COMPATIBILITY_CORPUS.md) · [support](docs/SUPPORT.md) · [1.0 readiness](docs/ONE_ZERO_READINESS.md) · [flagship validation](docs/FLAGSHIP_VALIDATION_WORKFLOW.md)
- Indoor autonomy: [multi-floor navigation](docs/MULTI_FLOOR_NAVIGATION.md)
- Physics: [height field terrain](docs/HEIGHT_FIELD_TERRAIN.md) · [collision bake](docs/COLLISION_BAKE.md) · [arm position control](docs/ARM_POSITION_CONTROL.md)
- Locomotion: [G1](docs/G1_LOCOMOTION.md)/[workbench](docs/G1_WORKBENCH_MISSION.md)/[splat bg](docs/G1_HEAD_SPLAT_BACKGROUND.md) · [Go2](docs/GO2_LOCOMOTION.md) · [frontier plan](docs/PLAN_LEGGED_LOCOMOTION_FRONTIER.md) · [sensors](docs/IMU_SIMULATION.md)
- Case studies: [Tsukuba](docs/TSUKUBA_CONFIRMATION_RUN.md)/[full](docs/TSUKUBA_FULL_RUN.md)/[3DGS bg](docs/TSUKUBA_3DGS_BACKGROUND.md) · [SSL 2v2](docs/SSL_SMALL_PITCH.md)/[adapter](docs/SSL_ADAPTER.md)
- [More demos](docs/DEMOS.md) · [Examples](examples/README.md) · [Changelog](CHANGELOG.md)

## License

Licensed under either the [Apache License 2.0](LICENSE-APACHE) or the [MIT license](LICENSE-MIT), at your option.
