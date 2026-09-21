# G1 backflip with optimization and contact simulation

This optional Python benchmark searches a finite set of maneuver parameters
using differential evolution and bounded local sweeps. It uses no RL training,
learned policy, external base wrench, or imposed floating-base trajectory.
MuJoCo integrates the complete free-base robot and ground contacts; the motors
receive joint targets and bounded effort.

The robot is RNE's existing 23-joint G1 URDF. This is an external contact-plant
benchmark, not yet a demonstration of the RNE/Rapier backend or real hardware.
It adds no dependency to a Rust crate.

## Partial specification screen (not hardware validation)

The saved GIF uses a **139 N m knee ceiling and disables self-collision**.
Unitree lists maximum knee torque of **90 N m for G1 and 120 N m for G1 EDU**
on its [official specification page](https://www.unitree.com/g1/).
The saved motion therefore does not demonstrate feasibility on either machine.

Two optional profiles, `scripts/fixtures/g1_backflip_screen.json` and
`g1_edu_backflip_screen.json`, enable robot self-collision, cap knee effort at
90/120 N m, select the URDF-declared mass (34.133858 kg), and sample joint target
and gain commands every 2 ms. The period follows the
[official low-level Python example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/low_level/g1_low_level_example.py).
One additional 2 ms command delay is an explicit test assumption, **not a measured
hardware latency**. Targets and gains are held between updates; flight/contact
state transitions are sampled on that clock. The implicit motor PD and idealized
speed envelope remain part of the continuously integrated plant.

Self-collision uses the URDF collision meshes (MuJoCo convex mesh collision),
including the foot mesh; the four sole points still handle ground contact.
MuJoCo's normal same-body/parent filtering remains. Any penetrating self-contact
fails the rollout immediately. Mesh approximations and this conservative gate
need comparison with the real geometry before interpreting contacts as measured
hardware interference.

Replaying the existing candidate at 0.125 ms gives:

| Change from the saved benchmark | Result |
|---|---|
| Only knee ceiling 120 N m | Fails; non-foot ground contact at 1.703 s |
| Only 2 ms command period, no delay | Passes the full five-second gate |
| Only self-collision enabled | Fails; hand–hand contact at 0.658 s |
| Combined G1 screen | Fails; hand–hand contact at 0.645 s |
| Combined G1 EDU screen | Fails; hand–hand contact at 0.652 s |

[Screen summaries](evidence/g1-contact-backflip/screening/) record each exact
profile and motion parameter set. These are dynamic rollouts, not geometric
replays of the GIF. Each summary can be reproduced using `--parameters` with
that file; the embedded profile is restored automatically. An explicit
`--profile` replaces the embedded profile.

```bash
# Exit 2 is expected for the current candidate under the stricter screen.
target/research/backflip-env/bin/python -I scripts/g1_backflip_search.py --generations 0 --parameters docs/evidence/g1-contact-backflip/candidate.json --profile scripts/fixtures/g1_edu_backflip_screen.json --output target/research/g1-edu-screen
```

A bounded EDU search improved takeoff upward speed to about 1.98 m/s, but
the full maneuver still failed on knee–torso contact. A candidate with optimized
arm spread avoided self-contact but hit the floor with its hands.
[Search evidence and termination status](evidence/g1-contact-backflip/README.md#bounded-edu-search-results)
are recorded; no stricter-profile backflip has passed.

Other joint torque/speed ceilings remain URDF assumptions; mass/COM, motor
power/current/thermal limits, gain limits, elastic transmission, state-estimation
errors and actual transport latency are not identified. Standard G1 development
access must not be inferred from this numerical profile. Neither profile is a
hardware controller or permission to execute the maneuver on a robot.

## Measured benchmark result

![G1 optimized backflip in the contact plant](media/unitree-g1-optimization-backflip.gif)

The saved controller passes at **0.125 ms and 0.0625 ms** integration steps.
Both five-second rollouts complete one backward revolution, land without any
non-foot ground contact, and remain upright for the final second. A replay
regression compares the complete recorded-state SHA-256 at 0.125 ms.

| Measurement | 0.125 ms | 0.0625 ms |
|---|---:|---:|
| Backward rotation | 360.040° | 360.040° |
| Continuous flight | 0.49875 s | 0.49713 s |
| Root rise above initial standing height | 0.18226 m | 0.18072 m |
| Maximum motor torque | 139 N m | 139 N m |
| Maximum joint-position excess | 0.01745 rad | 0.01751 rad |
| Maximum joint-speed / URDF rating | 1.03820 | 1.03826 |
| Minimum upright cosine in final second | 0.99999975 | 0.99999975 |

[Candidate, measurements and recordings](evidence/g1-contact-backflip/README.md)
include a rejected earlier controller: its coarse-step success did not survive
refinement. The final result is a benchmark demonstration under the model below;
two successful fine steps do not prove general robustness or hardware readiness.

## Reproduce

Keep at least 30 GiB free. The scripts refuse evaluations/rendering below this
reserve, save only the best search candidate and final recording, and do not
install packages themselves. A Python 3.12 environment can be prepared with:

```bash
python3 -m venv target/research/backflip-env
target/research/backflip-env/bin/python -I -m pip install --no-cache-dir -r scripts/g1-backflip-requirements.txt
target/research/backflip-env/bin/python -I -m unittest discover -s scripts -p test_g1_backflip_plant.py
```

Replay and render the pinned result (exit 0 means the complete gate passed):

```bash
target/research/backflip-env/bin/python -I scripts/g1_backflip_search.py --generations 0 --parameters docs/evidence/g1-contact-backflip/candidate.json --output target/research/g1-backflip
target/research/backflip-env/bin/python -I scripts/g1_backflip_search.py --generations 0 --dt-s .0000625 --parameters docs/evidence/g1-contact-backflip/candidate.json --output target/research/g1-backflip-fine
MUJOCO_GL=egl target/research/backflip-env/bin/python -I scripts/g1_backflip_render.py target/research/g1-backflip --output target/research/g1-backflip.gif
```

A headless OpenGL/EGL implementation is required only for GIF generation.
The simulation and tests require no renderer. Search can be restarted with
`--generations 15 --stage flip`; `--stage launch` freezes flight parameters and
`--stage flight` freezes launch timing and sagittal targets. Arm spread is
shared across phases and remains variable in both searches.
`--balance-kp` / `--balance-kd` support
bounded landing-gain sweeps. Optimization output always records failures too;
exit 2 means the complete backflip gate failed, including launch-only runs.
The pinned fine integration rates are offline numerical checks, not demonstrated
hardware control frequencies.

## Model and controller contract

- The checked manifest pins the URDF SHA-256, joint ordering, torque ceilings,
  initial pose, and four points per sole. The free base has no actuator. Gravity
  is 9.81 m/s²; the floor friction coefficient is 0.7.
- The `rne` mass policy reproduces the legacy 1 kg fallback for each of four
  inertial-less fixed sensor frames. MuJoCo's XML serialization rounds the total
  to 38.133858 kg. `declared` omits those added masses (34.133858 kg); results
  always identify the selected policy.
- Joint rotor inertia 0.01 kg m², damping 0.05 N m s/rad, and friction loss
  0.2 N m follow [Unitree's G1 23-DoF MJCF](https://github.com/unitreerobotics/unitree_mujoco/blob/ab03eec53238487c4a7c7f61dd8e29a2f66abc04/unitree_robots/g1/g1_23dof.xml).
  These quantities are absent from the URDF and are explicit additions.
- Motor effort never exceeds the manifest's constant ceilings (peak knee torque
  139 N m). Motoring effort tapers from 90% to 100% of the URDF velocity rating;
  overspeed requests braking within the same torque ceiling. This is a declared
  idealized actuator envelope, not an identified electrical motor model.
- Contacts use a 4 ms solver time constant. The final joint stops use 2 ms
  and activate 0.005 rad inside the URDF limits as a conservative margin.
  Both use MuJoCo's soft constraint solver. Position-limit
  excess must remain below 0.02 rad and speed below 1.05 times the rating. The
  numerical tolerances are checked at every integration step, including impact.
- In the original benchmark, only robot-ground collisions are enabled. Self-collision, gear elasticity,
  actuator latency, terrain variation, and hardware uncertainty are not covered.
- A deterministic state machine performs crouch, extension, tuck, opening,
  landing, and recovery. Touchdown and takeoff use measured foot contacts.
  Root pitch and velocity feedback adjust the ankle targets after landing.
  The integrator uses simulation steps, never wall-clock time.

The original 13 motion parameters are crouch knee, crouch lean, extension duration,
extension hip, extension ankle, tuck duration, tuck hip, tuck knee, landing knee,
extension shoulder, landing hip bias, opening angle, and crouch hip bias.
New searches also optimize a fourteenth parameter, symmetric shoulder roll
(arm spread), to allow avoiding hand–hand and arm–leg collisions. Saved 13-value
candidates retain their original arm posture.
Angles are radians and durations seconds. Landing feedback gains are recorded
separately. The random seed is fixed to 20260921. A feasible candidate is not
proof of globally optimal motion.

Success requires approximately one backward revolution, an airborne interval,
feet-only ground contact, finite states without solver resets/warnings, bounded
joint position/speed, and stable standing throughout the final second of a
five-second episode. GIF rendering refuses failed rollouts and renders the
recorded simulated configurations; it does not invent or interpolate a flip.

## Literature and OSS basis

- [Chignoli and Kim, Online Trajectory Optimization for Dynamic Aerial Motions
  of a Quadruped Robot](https://arxiv.org/abs/2110.06330) demonstrates optimized
  aerial maneuvers including flips. It supports the non-RL direction; it is not
  evidence that the same controller transfers to G1.
- [Crocoddyl paper](https://arxiv.org/abs/1909.04947) and
  [implementation](https://github.com/loco-3d/crocoddyl) provide a multi-contact
  optimal-control reference. No Crocoddyl dependency or implementation is added
  to RNE core by this benchmark.
- [Unitree's MuJoCo repository](https://github.com/unitreerobotics/unitree_mujoco)
  supplies the explicit motor-model convention above. Geometry and inertias
  come from the URDF already in this repository.

The search here evaluates low-dimensional trajectory/controller parameters in
the contact plant directly. It is distinct from solving a whole-body NLP and
assuming that a small transcription residual guarantees a trackable landing.
