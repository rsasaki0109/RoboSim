# Shared-engine regression repairs

The quaternion comparison removes only f64 normalization temporarily. It
restores the ten old failures but breaks seven checks under the prior lift
0.3 Nm / robust-turn 0.85 calibration (369 pass, 7 fail, 12 ignored). The
experiment is reverted; production retains normalized orientations.

Corrected procedural wheel geometry, fixed low-friction supports and force-based
1 Nm velocity motors pass 37 existing diff-drive/agent checks and four robot
geometry/kinematics tests, plus release all-target Clippy with `-D warnings`.
The final lift claw cap of 0.1 Nm passes 23 tests matching `lift` and the separate
place-observation test. Example 31 `--smoke` carries 1.06 m and releases at
(0.58, 0.03, -0.86). All existing positive behavior assertions are unchanged.

The force sweep used a temporary test-only override, now removed. The final
0.1 Nm tests and smoke use the production constant without an environment
override. These are targeted checks, not a claim that full workspace CI passes.
The immutable G1 backflip producer and its physical gates are unchanged.

Final production calibration passes all 376 `rne_ai` unit tests (zero failures,
12 pre-existing ignored tests) and release all-target Clippy for `rne_ai` and
`rne_robot`. Probe-only environment controls were removed before this run.
The selected gains and rejected bounded sweeps are preserved alongside the logs.
Positive behavior assertions are unchanged. The hand-designed gain-25 negative
steering experiment now allows a fall as evidence of rejection; it must never
be accepted as an upright sustained turn. Baseline and other hand-pattern
upright/travel requirements remain in force. G1 backflip gates are untouched.

The corrected wheel layout also exposed that the reference camera had seen a
wheel rather than the intended scene. After the correction, the old capture
was all far-plane depth (identical failing golden hash on Ubuntu and Windows).
Example 73 now renders scene colliders/obstacles with a wall ahead of its +X
camera and rejects empty or non-finite reference depth. Dataset v3 preserves
471 records, zero drops and success termination; its 0.0050001144 m maximum
depth error passes the unchanged 0.01 m tolerance. A fresh repeated capture
matches the v3 golden exactly. The v1/v2 goldens are preserved unchanged.
The guard test and release all-target example Clippy pass; cross-platform CI
verification of v3 remains pending.
