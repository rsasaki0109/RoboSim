# Robot workbench

The local workbench combines native joint control, saved poses, a floor/obstacle
editor, RGB/depth/LiDAR inspection, and URDF/MJCF loading in one browser window.
Rapier runs the physics and wgpu renders the images on the host. The browser
displays those images and sends commands; it does not synthesize robot motion or
sensor readings.

```bash
cargo run --release --locked -p robot_workbench
```

Open the printed loopback URL (normally `http://127.0.0.1:8765`). The default
robot is the already-vendored SO-101, with its authored mesh colors. No robot
pack, web framework, browser renderer, or JavaScript dependency download is
needed. A working native wgpu adapter is needed for the interactive host.

![SO-101 workbench with joint controls and native sensor views](media/robot-workbench.png)

## Control a robot and save a pose

Set named joint targets with the sliders or numeric fields, then press Play.
Targets and measured positions are shown separately; revolute coordinates use
radians and prismatic coordinates use metres. Pause stops new step requests.
Single Step advances one `4,166,667 ns` step; Play requests bounded batches of
four such steps, so browser redraw timing never becomes simulation time. The
host does not promise real-time playback speed.

The servo ramps its commanded position using the model's velocity limit and
caps motor effort at its effort limit. These are command/effort limits, not a
claim that contact or gravity cannot produce a larger measured velocity. When
the source has no actuator specification (including the current MJCF importer),
the UI labels the preview defaults: 1 rad/s and 10 N·m, or 1 m/s and 10 N for a
linear axis. Continuous joints have a ±2π target-entry window; this is not a
physical joint stop. Invalid, unknown, missing, non-finite, out-of-range or wrong-unit
targets are rejected before replacing the current targets.

Name a pose and choose **Save targets**. Applying a saved pose changes targets;
the robot moves when stepping resumes. A pose stores named, explicitly typed
angular/linear coordinates and the source robot's SHA-256 identity. It cannot
silently load onto a different source model.

## Edit the scene and retain a project

Select the floor or an obstacle, edit its position, dimensions and color, then
apply. Add Box creates another static obstacle; Delete removes the selected
object. Floors are ordinary thin boxes. Scene edits explicitly reset the robot
and simulation clock, so a stale physics world cannot disagree with the edited
geometry. The same colliders are used by physics and LiDAR.

Save JSON downloads a versioned `.rne.workbench.json` project containing the
source XML, target positions, saved poses, model orientation/inertia choice and
scene objects. Load JSON validates the candidate and constructs its physics
world and mesh resources before swapping live state. A rejected import leaves
the old project and physics state intact. File operations stay in the browser's
normal upload/download flow; the server does not overwrite project files.

## Inspect sensors

- RGB and depth use the same fixed 160 × 120 observation camera. Hover over a
  depth pixel to read metres. Black pixels are out of range, not zero distance.
- LiDAR casts 180 horizontal rays at world height 0.5 m with a 10 m range.
  Its top-down plot has 1 m rings; `null` ranges mean no return. An empty valid
  scan remains distinguishable from an error.
- Camera and LiDAR share the exact simulation timestamp of the displayed
  state. These workbench sensors are instantaneous and noiseless, with zero
  simulated transport latency. They are world-mounted inspection sensors,
  independent of the orbit camera used by the main robot view.

## Load another robot

The model picker offers SO-101, a prismatic URDF slider, and a two-joint MJCF
arm. The file picker accepts URDF or MJCF source XML. For meshes, set the asset
root to the robot package directory:

```bash
cargo run --release --locked -p robot_workbench -- \
  --robot assets/robots/so101/so101.urdf \
  --asset-root assets/robots/so101 --port 8765
```

Select Z-up to convert a Z-up model into the workbench's Y-up world; the SO-101
preset already declares this conversion. Mesh references must resolve beneath
the selected asset root. Mesh files are not embedded in the saved project, so
reopening a mesh project on another machine also requires that asset root.

The default **simple preview** mode deliberately uses the importer's existing
default link masses and collision-derived inertia. Select **URDF mass/inertia**
for models with valid declared inertials. Invalid tensors are rejected rather
than repaired silently. The vendored SO-101 has a near-massless dummy frame with
a zero inertia tensor, so its default preset uses simple preview. The workbench
fixes the robot base and disables robot self-collision; it is an interactive
control/model inspection tool, not a calibrated hardware or locomotion proof.

MJCF support is exactly the existing strict `rne_mjcf` subset: hinge/slide
joints and supported primitive/mesh geometry. It does not import a MuJoCo
actuator model, derive authored inertials, or support free/ball joints and
arbitrary MJCF features. Physics mimic joints are also rejected by this host.
Errors are displayed without discarding the previously loaded scene.

Inputs are bounded: approximately 4 MiB per project/request, 2 MiB source XML,
128 objects/poses, 256 controllable joints, 512 links/joints and 32 MiB per mesh.
The host binds only to `127.0.0.1`, accepts same-origin JSON commands, and keeps
simulation state in one owned application instance.

## Validation

Model, pose, editing, native control, replay and LiDAR checks need no GPU:

```bash
cargo test --locked -p robot_workbench --no-default-features
cargo run --release --locked -p robot_workbench --no-default-features -- --smoke
cargo run --release --locked -p robot_workbench --no-default-features -- \
  --check-project robot.rne.workbench.json
```

The project checker above is for primitive-only projects; mesh projects need
the interactive host's explicit asset root. `xtask ci-headless` includes the
workbench's headless tests and smoke. Workspace lint/tests also compile its
interactive code and HTTP boundary tests.

With the server running, Playwright and an existing Chrome/Chromium installation
can exercise real UI interactions. This script downloads nothing:

```bash
python3 scripts/test_robot_workbench_browser.py \
  --url http://127.0.0.1:8765 --browser /usr/bin/google-chrome \
  --output /tmp/robot-workbench-checks
```

The browser check covers SO-101 motion, pose save/apply, scene editing,
project export/reload, rejected edits preserving physics state, URDF/MJCF file
uploads, sensor pixels, and desktop/mobile layout. Its screenshots and JSON
files are small diagnostic artifacts; normal use records no video.

All workbench code lives in the non-published application example
`examples/116_robot_workbench`. No public core API, frozen compatibility
baseline, ROS dependency, or rendering requirement for simulation tests changes.
