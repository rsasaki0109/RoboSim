# Lift-capable mobile pick-and-place

The `mm_mobile_lift` asset is an engine-native mobile manipulator: the base,
vertical carriage, arm, wrist yaw, and prismatic parallel jaws are URDF entities
with physics-backed joints. The shipped headless workflow is:

```text
navigate → approach → grasp → lift → transport → place
```

Run the flagship smoke demo with:

```text
cargo run -p mobile_lift_friction_pick_place --example 72_mobile_lift_friction_pick_place -- --smoke
```

The policy selects `GraspMode::Friction`. The payload remains a free dynamic rigid
body; no `FixedJointDesc` is inserted for the grasp. Finger normal force and the
lower payload/pad friction coefficient bound the tangential carry aid, so a
low-friction payload slips and is classified rather than being geometrically
welded. The wrist camera publishes deterministic RGB and linear-depth frames to
the DataBus, and the episode checkpoint captures those latest frames for replay.
The robot asset opts into its URDF link masses explicitly; other imported URDFs
retain the legacy mass defaults unless `use_declared_inertial_masses` is enabled.

`IkMobileLiftPickPlacePolicy::failure_class` reports deterministic categories such
as grasp timeout, grasp slip, lift-clearance timeout, transport timeout, and
release timeout. The same policy and episode are exposed through the Python
`MobileManipulatorSim("mm_mobile_lift")` and vectorized checkpoint APIs. For the
full scripted path, use `MobileManipulatorEpisode("mobile_lift_place")`, select
`set_grasp_mode("friction")`, and pass `IkMobileLiftPickPlacePolicy.act()` to
`step_action()`. The returned action preserves the absolute lift target and the
linear gripper velocity. `MobileManipulatorObservation` also exposes the
nearest-depth wrist pixel and camera-frame offsets (`wrist_target_*`).
