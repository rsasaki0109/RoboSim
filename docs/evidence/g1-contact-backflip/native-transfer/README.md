# Native Rapier transfer: negative evidence

These are diagnostic failures, not backflip success evidence. Run from the
repository root using example 114 `--native-probe`; use `--native-dt-us 125`
for direct effort and `--native-dt-us 500 --native-implicit` for the native
position-motor diagnostic. See `docs/G1_CONTACT_BACKFLIP.md` for model and
controller differences.

Time zero is the start of the maneuver after one second of motor-only standing
preparation. The direct-effort run leaves valid bounds at -0.789875 s; the
implicit run stands through preparation and takes off at 0.837 s, but leaves
valid bounds at 0.8945 s. Final-second metrics are null because neither run
reaches that interval. `qualified_backflip` remains false.

Neither experiment writes the base pose or velocity. The recorded model uses
RoboSim's existing primitive-collision scene with self-collision disabled. The
implicit motor diagnostic does not enforce the external motor-speed taper.
The failure trigger is a numerical/scene-bound guard, not a physical contact
qualification gate. A stable native plant/controller baseline is still needed.

Validation for this follow-up: workspace formatting check, targeted release
Clippy with `-D warnings`, the quaternion conversion unit test, all 500
recorded-frame projections, and rejection of a changed recording checksum
passed. The direct-effort failure reproduced byte-for-byte from the isolated
PR tree. Full workspace tests and xtask suites were not rerun to retain the
user-requested 30 GiB disk reserve.
