# mm_mobile_lift visual contract

`rne_assets` accepts a versioned, visual-only manifest for the
`mm_mobile_lift` robot. The manifest maps the ten URDF links to relative mesh
files and enforces path, scale, material, texture, and triangle budgets.

The contract does not replace the robot URDF: collision geometry, joints,
limits, and inertial values remain owned by the physics asset. Runtime visual
overlay application is intentionally a separate integration step.
