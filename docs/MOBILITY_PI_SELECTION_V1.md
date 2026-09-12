# Fixed sensor policy parameter selection

Status: implemented and CI-validated. This is finite PI parameter selection in
simulation, not a learned perception policy or a sim-to-real result.

`observed::fixed_evaluation::tuning::select_fixed_pi_policy` evaluates 1--32 unique
gain candidates on 1--64 explicit training seeds, selects the smallest mean
integrated absolute speed error (m), then executes only the winner on 1--64
disjoint held-out seeds. Lists must be strictly increasing. Input validation runs
before any factory call. Exact ties select the earliest candidate. Any execution
error makes that candidate ineligible, but unsuccessful final-speed gates remain
scored and recorded. If all candidates error, no held-out rollout is executed.

The fixed horizon makes this objective equivalent to maximizing summed TaskSpec
reward: the constant per-step cost is identical for every complete candidate.
No horizon, acceptance threshold, plant parameter, sensor calibration or voltage
limit changes during selection. Policy gains are separate from reset metadata;
the latter's PI fields still describe the old reference controller, not the
candidate. Each controller is fresh and sees only sensor-derived observations.
The offline selector may use privileged reward, not physical state as policy input.

The report retains all candidate/seed outcomes and binds each completed full
evaluation (commands, reset, diagnostics and final physics hash) by SHA-256.
It is diagnostic evidence, not a signed artifact or a common Failure Capsule.
Factory/solver errors are preserved; a caller panic still unwinds. Backends must
be independent and have stable manifests. This is a sequential reference, not
the persistent parallel training adapter that remains planned.

Regression experiment is fixed before execution: candidates `(Kp, Ki)` are
`(15,20), (30,20), (15,40), (30,40)` in V s/m and V/m. Training seeds are 101--104;
held-out seeds are 1001--1004. The chosen gains also run on MuJoCo without retuning.
Held-out observations do not feed selection. These small, reusable regression
sets are not an independent final generalization test; broader frozen evaluation
and real-log validation are still required.

Initial experiment: training selected `(15,40)` with mean integral 0.448371 m,
versus 0.801092 m for the unchanged `(15,20)` baseline. Held-out final speeds on
Rapier were 1.038606, 1.053616, 0.947541, 1.035004 m/s in seed order. MuJoCo speeds
were 1.038689, 1.054525, 0.947021, 1.034220 m/s with the same selected gains.
All eight final-speed gates passed; this is not a whole-task or real-world verdict.
Evidence: `E:/RNE-build/m3c-sensor/pi-selection-tests.log`. Candidate selection
does not overwrite the existing reference policy or erase its failed cases.

The separation follows the [scikit-learn data-leakage guidance](https://scikit-learn.org/stable/common_pitfalls.html#data-leakage):
test data must not influence model choices. The implementation tests that changing
held-out seeds leaves candidate scores and the selected index unchanged. It does
not promise that a user repeatedly inspecting these results cannot overfit them.

Validation (2026-09-07): all 14 fixed-evaluator tests and feature-enabled Clippy
passed. The MuJoCo-enabled mobility suite passed 97 library tests and one CLI
test. Full `xtask ci` exited with code zero, including workspace tests, headless,
OSS parity, fuzz smoke and Behavior CI (10/10 seeds). Logs are on the external
drive: `E:/RNE-build/m3c-sensor/pi-selection-final-tests.log`,
`pi-selection-final-clippy.log`, `pi-selection-mujoco-tests.log`, and
`pi-selection-ci.log` in that same directory.
