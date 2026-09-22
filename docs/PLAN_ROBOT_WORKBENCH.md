# Robot workbench

Status: implementation and local validation complete on `feat/robot-workbench`;
repository-wide validation runs in the pull request CI.

## Goal

One local RoboSim workspace for joint control and saved poses, floor/obstacle
editing, RGB/depth/LiDAR inspection, and loading another URDF or supported MJCF
robot. The existing native importers, physics backend and renderer provide the
robot model and measurements; browser controls are an application boundary.

## Delivery checklist

- [x] Validated, versioned project and pose persistence; rejected edits leave the
  previous project intact.
- [x] Joint sliders with model limits, measured/target positions, and save/load
  poses. Explicit fixed-step play, pause, step and reset.
- [x] Add/edit/remove floors and obstacles; save/reload the complete project.
- [x] Robot view plus timestamped RGB, metric depth and LiDAR views from the
  actual scene. Distinguish missing data from valid empty observations.
- [x] Load URDF and the existing supported MJCF subset, with useful import
  errors. Exercise two distinct bundled models and a file import.
- [x] Headless model/control/edit tests, browser interaction checks, runnable
  end-to-end smoke and usage documentation.
- Required repository validation: tracked by the pull request CI checks.

## Boundaries and resource budget

The workbench is a non-published application example, not a new public core API.
No renderer becomes necessary for model/control tests. Simulation advances by
`SimClock` and explicit fixed steps, independently of browser redraw timing.
Project edits and model replacement are validated before changing live state.
Joint commands respect the imported model's limits. Saved poses retain joint
names and angular/linear units and cannot silently apply to a different model.

Prefer the existing assets and dependencies; do not download new robot packs.
Keep generated logs bounded and avoid recording video during normal work.
Use a bounded memory-backed build directory where practical, monitor root disk
space, and preserve the user's dirty original checkout. Record unsupported
importer features as errors rather than claiming complete MJCF support.

## Local validation evidence

- Default-feature tests: 8 passed; CPU-only tests: 7 passed.
- Default and CPU-only Clippy pass with `--all-targets -- -D warnings`.
- CPU-only smoke exercises actual URDF and MJCF joint motion, saved poses,
  scene extraction and native LiDAR; replay tests compare stable hashes.
- Chrome/Playwright exercises SO-101 motion, pose save/apply, obstacle editing,
  project export/reload, atomic rejection, both file formats, native sensor
  data and desktop/mobile layout. See `media/robot-workbench.png`.
- All build artifacts and browser temporary data were placed on bounded tmpfs;
  the original working checkout was not modified.
